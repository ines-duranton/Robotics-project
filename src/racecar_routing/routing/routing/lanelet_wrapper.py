import lanelet2
from lanelet2.core import BasicPoint2d, BasicPoint3d
from lanelet2.projection import LocalCartesianProjector
from lanelet2.geometry import to2D, toArcCoordinates, fromArcCoordinates, ArcCoordinates, distanceToCenterline2d

import numpy as np
import rclpy
from rclpy.node import Node

from scipy.ndimage import gaussian_filter1d
import random

from functools import lru_cache


'''
This class is a wrapper for lanelet2 objects.
It uses the lanelet2 Python API to provide a more convenient interface using generic data types.
Now fully migrated to ROS 2 and compatible with ROS 2 node-based parameter handling.
'''
class LaneletWrapper:
    def __init__(self, osm_file, node: Node) -> None:
        # Store the node handle for logging and parameter access
        self.lane_change_cost = node.lane_change_cost
        self.node = node

        if self.node:
            self.node.get_logger().info(f"Loading map from {osm_file} with lane_change_cost={self.lane_change_cost}")


        # Create a local cartesian projector from (0, 0, 0)
        self.projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))

        # Load lanelet map
        self.lanelet_map = self.load_lanelet_map(osm_file)
        self.lanelet_layer = self.lanelet_map.laneletLayer

        # Set up traffic rules and routing graph
        self.traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle
        )

        "Takes into account Distance and Travel Time when calculating routing, also adds a static lane change cost"
        routing_cost = [
            lanelet2.routing.RoutingCostDistance(self.lane_change_cost),
            lanelet2.routing.RoutingCostTravelTime(self.lane_change_cost)
        ]

        self.routing_graph = lanelet2.routing.RoutingGraph(
            self.lanelet_map, self.traffic_rules, routing_cost)

        # Pre-cache all centerlines to avoid lazy evaluation delays
        self.warmup()

    def find_lanelet_by_xy(self, pt):
        '''
        Given a 2D point, find the nearest lanelet in the map.
        Returns:
            dis: distance to the nearest lanelet (0 if inside)
            lanelet: nearest lanelet object
        '''
        return lanelet2.geometry.findNearest(self.lanelet_layer, pt, 1)[0]
    
    def get_closest_lanelet(self, pose, check_psi: bool=False, psi_weight = 1):
        '''
        Get the closest lanelet to the given pose
        Parameters:
            pose: array, [x, y, (optional) psi]
            check_psi: bool, whether to check heading difference
        Returns:
            closest_lanelet: PyLaneLet, closest lanelet
            lanelet_s: float, normalized position on the centerline
            
        '''
        min_dist = float('inf')
        closest_lanelet = None
        lanelet_s = None
        if check_psi:
            psi = pose[2]

        
        point = BasicPoint2d(float(pose[0]), float(pose[1]))
        
        for lanelet in self.lanelet_layer:
            cl = to2D(lanelet.centerline)
            d = lanelet2.geometry.distanceToCenterline2d(lanelet, point)
            s = lanelet2.geometry.toArcCoordinates(cl, point)
            dist = abs(d)
            # check heading
            if check_psi:
                projected_point = lanelet2.geometry.project(cl, point)
                arc_coord = ArcCoordinates()
                arc_coord.dis = 0.0
                arc_coord.length = 0.0
                zero_point = lanelet2.geometry.fromArcCoordinates(cl, arc_coord)
                ref_psi = np.arctan2(projected_point.y - zero_point.y, projected_point.x - zero_point.x)
                psi_dist = psi - ref_psi
                psi_dist = psi_weight * np.abs(np.arctan2(np.sin(psi_dist), np.cos(psi_dist)))
            else: 
                psi_dist = 0
            
            total_dist = dist + psi_dist
            
            if total_dist < min_dist:
                min_dist = total_dist
                closest_lanelet = lanelet
                lanelet_s = s
            
        return closest_lanelet, lanelet_s
    
    #note, this does not take into account psi_distance
    def get_shortest_path(self, start_pose, end_pose, start_bool: bool = False, end_bool: bool = False, allow_lane_change: bool = True):
        '''
        Computes the shortest path from start (x_start, y_start) to end (x_end, y_end)
        using the Lanelet2 routing graph.
        Returns:
            path_centerline: list of waypoints representing the path centerline
        '''
        path_centerline = []

        # Find nearest lanelets to start and end
        start_lanelet, start_s = self.get_closest_lanelet(start_pose, check_psi=start_bool)
        end_lanelet, end_s = self.get_closest_lanelet(end_pose, check_psi=end_bool)


        #returns a laneletRelation
        """
        determines the relationship between 2 lanelets
        *left * right: (via lane change)
        *adjacent left *adjacent right: neighbors but not accessable via lane change
        *succeeding: between succeeding lanelets
        *conflicting: intersecting lanelets
        *area: reachable area to lanelet/area relation, not needed in our case
        Thus will return a list of all relations between the start and end node
        """
        relation = self.routing_graph.routingRelation(start_lanelet, end_lanelet, True)

        # Handle special case when start and end are in the same or neighboring lanelet
        if start_lanelet.id == end_lanelet.id or \
            relation in [lanelet2.routing.RelationType.Left, lanelet2.routing.RelationType.Right]:
            
            start_point = self.project_xy_to_linestring(start_pose[0], start_pose[1], start_lanelet.centerline)
            cl = to2D(start_lanelet.centerline)
            cl_length = lanelet2.geometry.length(cl)
            dis_start = start_s.length / cl_length
            dis_end = end_s.length / cl_length
            if dis_end < dis_start:
                path_centerline.extend(self.get_path_centerline([start_lanelet], start_point, None, False))
                start_lanelet = self.routing_graph.following(start_lanelet, False)[0]
                start_point = None
            elif dis_end == dis_start:
                return []  # Same point, no path needed

        # Get the shortest path using the routing graph
        path = self.routing_graph.shortestPath(start_lanelet, end_lanelet, 0, allow_lane_change)
        raw_centerline = self.get_path_centerline(path, start_pose[:2], end_pose[:2], allow_lane_change)
        path_centerline.extend(raw_centerline)

        centerline = np.array(path_centerline)
        _, idx = np.unique(np.round(centerline[:,:2], 3), axis=0, return_index=True)
        centerline_unique = centerline[np.sort(idx), :]
        return centerline_unique

    """
    Given a piece path of lanelets, smooth the path
    path here is a list of x,y points
    """
    def smooth_path(self, path, k = 0.01):
        xs = np.array([p[0] for p in path])
        ys = np.array([p[1] for p in path])
        xs_s = gaussian_filter1d(xs, sigma=k, mode='nearest')
        ys_s = gaussian_filter1d(ys, sigma=k, mode='nearest')

        #preserve the old endpoints
        xs_s[0], ys_s[0] = xs[0], ys[0]
        xs_s[-1], ys_s[-1] = xs[-1], ys[-1]

        smoothed = []
        for i, p in enumerate(path):
            x_new = float(xs_s[i])
            y_new = float(ys_s[i])
            smoothed.append([x_new, y_new, p[2], p[3], p[4]])

        return smoothed
        


    def get_path_centerline(self, path, start_point=None, end_point=None, allow_lane_change=True):
        '''
        Converts a list of lanelets into a full centerline path.
        If start_point or end_point are provided, trims the path accordingly.
        '''
        path_centerline = []
        num_lanelet = len(path)

        prev_start = 0
        if start_point is not None:
            cl = to2D(path[0].centerline)
            cl_length = lanelet2.geometry.length(cl)
            prev_start = self.get_dis_from_point(cl, start_point[0], start_point[1]) / cl_length
            prev_start = min(0.98, prev_start)

        last_lanelet = path[-1]
        if end_point is not None:
            cl = to2D(last_lanelet.centerline)
            cl_length = lanelet2.geometry.length(cl)
            dis_end = self.get_dis_from_point(cl, end_point[0], end_point[1]) / cl_length
        else:
            dis_end = 1

        # Build centerline for each lanelet segment
        for i in range(num_lanelet - 1):
            cur_lanelet = path[i]
            next_lanelet = path[i + 1]
            relation = self.routing_graph.routingRelation(cur_lanelet, next_lanelet, True)

            if relation in [lanelet2.routing.RelationType.Left, lanelet2.routing.RelationType.Right]:
                if i == num_lanelet - 2:
                    cur_end = (dis_end - prev_start) * 0.3 + prev_start
                    next_start = (dis_end - prev_start) * 0.7 + prev_start
                else:
                    cur_end = (0.98 - prev_start) * 0.3 + prev_start
                    next_start = (0.98 - prev_start) * 0.7 + prev_start
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, cur_end, False, allow_lane_change))
                prev_start = next_start
            else:
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, 0.99, False, allow_lane_change))
                prev_start = 0

        path_centerline.extend(self.get_centerline_section(last_lanelet, prev_start, dis_end, True))
        return path_centerline
    

    def get_centerline_section(self, lanelet, norm_dis_start, norm_dis_end, include_end_point=False, allow_lane_change=True):
        '''
        Extract a segment of a lanelet's centerline, normalized from 0.0 to 1.0.
        Also includes lane width and speed limit information per point.
        '''
        centerline_section = []
        centerline = to2D(lanelet.centerline)
        length = lanelet2.geometry.length(centerline)
        speed_limit = self.get_lanelet_speed_limit(lanelet)

        assert norm_dis_start * norm_dis_end >= 0, "Start and end distances must have same sign"
        if abs(norm_dis_start) > abs(norm_dis_end):
            self.node.get_logger().warn(f"Start distance {norm_dis_start} must be less than end distance {norm_dis_end}")
            return centerline_section

        if norm_dis_start < 0:
            centerline = centerline.invert()

        dis_start = max(abs(norm_dis_start) * length, 0)
        dis_end = min(abs(norm_dis_end) * length, length)
        num_points = len(centerline) - int(include_end_point)

        # Add interpolated start point
        arc_coord = ArcCoordinates()
        arc_coord.dis = 0.0
        arc_coord.length = dis_start
        pt_start = fromArcCoordinates(centerline, arc_coord)
        width_left, width_right = self.get_lane_width(pt_start, lanelet, allow_lane_change)
        centerline_section.append([pt_start.x, pt_start.y, width_left, width_right, speed_limit])

        for i in range(num_points):
            pt = centerline[i]
            pt_len = self.get_dis_from_point(centerline, pt.x, pt.y)
            if pt_len <= dis_start:
                continue
            elif pt_len >= dis_end:
                break
            width_left, width_right = self.get_lane_width(pt, lanelet, allow_lane_change)
            centerline_section.append([pt.x, pt.y, width_left, width_right, speed_limit])

        if include_end_point:
            arc_coord.length = dis_end
            pt_end = fromArcCoordinates(centerline, arc_coord)
            width_left, width_right = self.get_lane_width(pt_end, lanelet, allow_lane_change)
            centerline_section.append([pt_end.x, pt_end.y, width_left, width_right, speed_limit])
        return centerline_section

    def get_lane_width(self, point, lanelet, allow_lane_change=True):
        '''
        Computes width from the center point to the left and right bounds of the lanelet.
        If lane changing is allowed, uses bounds of neighboring lanelets if available.
        '''
        left_bound = lanelet.leftBound
        right_bound = lanelet.rightBound

        if allow_lane_change:
            left_lanelet = self.routing_graph.left(lanelet)
            right_lanelet = self.routing_graph.right(lanelet)
            if left_lanelet is not None:
                left_bound = left_lanelet.leftBound
            if right_lanelet is not None:
                right_bound = right_lanelet.rightBound

        left_bound = to2D(left_bound)
        right_bound = to2D(right_bound)
        basic_point = BasicPoint2d(point.x, point.y)

        width_left = lanelet2.geometry.distance(left_bound, basic_point)
        width_right = lanelet2.geometry.distance(right_bound, basic_point)
        return width_left, width_right

    def get_random_waypoint(self, with_neighbors: bool = False):
        '''
        Randomly select a (valid) waypoint on the map
        Return:
            waypoint: [x,y,psi]
        '''
        
        # Randomly select a lanelet
        if with_neighbors:
            has_neighbor = False
            while has_neighbor == False:
                lanelet_id = random.choice(list(ll.id for ll in self.lanelet_layer))
                lanelet = self.lanelet_layer[lanelet_id]
                left = self.routing_graph.left(lanelet)
                right = self.routing_graph.right(lanelet)
                if left or right:
                    has_neighbor = True
                s = np.random.uniform(0.2,0.9)
        else:
            lanelet_id = random.choice((list(ll.id for ll in self.lanelet_layer)))
            s = np.random.uniform(0.2, 1.0)
        terminal_lanelet = self.lanelet_layer[lanelet_id]
        waypoint = self.get_point_at_dis(terminal_lanelet.centerline, s)
        heading_point = self.get_point_at_dis(terminal_lanelet.centerline, (s - 0.05))
        dx = waypoint.x - heading_point.x
        dy = waypoint.y - heading_point.y
        psi = np.arctan2(dy, dx)
        
        return waypoint.x, waypoint.y, psi
    
    def get_reachable_path(self, start_lanelet, start_s, d_max, allow_lane_change=True):
        """
        Compute all reachable paths (as lists of lanelets) from a starting lanelet,
        up to a maximum arc-distance d_max (meters).
        
        Args:
            start_lanelet:     lanelet2.core.Lanelet
            start_s:           normalized arc coordinate on start lanelet centerline (0–1)
            d_max:             max distance allowed for exploration
            allow_lane_change: whether lateral transitions are allowed

        Returns:
            list of centerline numpy arrays (N×2)
        """

        # Precompute lanelet lengths
        def lanelet_length(ll):
            return lanelet2.geometry.length(to2D(ll.centerline))

        start_len = lanelet_length(start_lanelet)

        start_offset = -start_s * start_len   # negative: distance remaining in start LL

        @lru_cache(maxsize=None)
        def explore(cur_id, dist_so_far, allow_lc):
            """
            Recursive memoized search for reachable lanelet sequences.
            Returns: list of routes (each a list of lanelet IDs)
            """
            ll = self.lanelet_layer[cur_id]
            results = []

            # ----- 1) Lane changes -----
            if allow_lc and allow_lane_change:
                left = self.routing_graph.left(ll)
                if left is not None:
                    left_id = left.id
                    for sub in explore(left_id, dist_so_far + 0.5, False):
                        results.append([cur_id] + sub)

                right = self.routing_graph.right(ll)
                if right is not None:
                    right_id = right.id
                    for sub in explore(right_id, dist_so_far + 0.5, False):
                        results.append([cur_id] + sub)

            # ----- 2) Forward successor(s) -----
            successors = self.routing_graph.following(ll, allow_lane_change)
            ll_length = lanelet_length(ll)
            new_dist = dist_so_far + ll_length

            # If exceeding max distance, terminal route
            if new_dist >= d_max:
                results.append([cur_id])
                return results

            for nxt in successors:
                nxt_id = nxt.id
                for sub in explore(nxt_id, new_dist, allow_lane_change):
                    results.append([cur_id] + sub)

            return results

        # ----- Start recursive exploration -----
        routes = explore(start_lanelet.id, start_offset, allow_lane_change)

        # ----- Convert each route to a centerline -----
        centerline_list = []
        for route in routes:
            ll_objects = [self.lanelet_layer[rid] for rid in route]
            centerline_points = self.get_route_centerline(ll_objects, start_s)
            centerline_list.append(centerline_points)

        return centerline_list

    def get_route_centerline(self, route_lanelets, start_s):
        """
        Convert a list of lanelet objects into a single concatenated centerline.
        """
        centerline_list = []

        for i, ll in enumerate(route_lanelets):
            s0 = start_s if i == 0 else 0.0
            s1 = 1.0

            is_last = (i == len(route_lanelets) - 1)

            pts = self.get_centerline_section(
                ll,
                norm_dis_start=s0,
                norm_dis_end=s1,
                include_end_point=is_last
            )

            for p in pts:
                centerline_list.append([p[0], p[1]])  # x,y only

        return np.array(centerline_list)



    def normalized_arc_distance(self, lanelet, pt):
        """
        Computes the normalized projection of point as arc distance s
        """
        cl = to2D(lanelet.centerline)
        point = BasicPoint2d(pt[0], pt[1])
        d = lanelet2.geometry.distanceToCenterline2d(lanelet, point)
        arc = lanelet2.geometry.toArcCoordinates(cl, point)
        lane_length = LaneletWrapper.get_lanelet_length(lanelet)

        s = arc.length / lane_length if lane_length > 0 else 0.0
        s = min(max(s, 0.0), 0.999999)   # clamp to [0, 1)

        return d, s

    def distance_to_centerline(self, lanelet, pose):
        """
        Computes minimum euclidean distance from pose to centerline of lanelet
        """
        point = BasicPoint2d(pose[0], pose[1])
        return lanelet2.geometry.distanceToCenterline2d(lanelet, point)

    @staticmethod
    def get_dis_from_point(line_string, x, y):
        '''
        Compute the length along a line string up to the projection of a 2D point.
        '''
        pt = BasicPoint2d(x, y)
        return toArcCoordinates(line_string, pt).length

    @staticmethod
    def get_lanelet_length(lanelet):
        '''Return the total 2D centerline length of a lanelet.'''
        return lanelet2.geometry.length2d(lanelet)

    @staticmethod
    def get_linestring_length(line_string):
        '''Return the total length of a 3D linestring.'''
        return lanelet2.geometry.length(line_string)

    #@staticmethod
    def get_lanelet_speed_limit(self, lanelet):
        '''Return the speed limit for the given lanelet, defaulting to 2.0 if not present.'''
        if 'speed' in lanelet.attributes:
            return float(lanelet.attributes["speed"])
        else:
            return 2.0

    @staticmethod
    def get_point_at_dis(line_string, dist):
        '''Get a point at a given arc distance along a line string (interpolated).'''
        return lanelet2.geometry.interpolatedPointAtDistance(line_string, dist)

    @staticmethod
    def get_point_at_normalized_dis(line_string, dist_norm):
        '''Same as above, but with normalized distance from 0 to 1.'''
        dis = dist_norm * lanelet2.geometry.length(line_string)
        return lanelet2.geometry.interpolatedPointAtDistance(line_string, dis)

    @staticmethod
    def project_xy_to_linestring(x, y, line_string):
        '''Project a 2D point onto a 3D linestring and return the closest projected x, y.'''
        pt = BasicPoint3d(float(x), float(y), 0.0)
        pt_project = lanelet2.geometry.project(line_string, pt)
        return pt_project.x, pt_project.y

    def load_lanelet_map(self, osm_file):
        '''Load a Lanelet2 map from .osm using the local projector.'''
        return lanelet2.io.load(osm_file, self.projector)

    def save_lanelet_map(self, lanelet_map, osm_file):
        '''Save a Lanelet2 map to .osm using the local projector.'''
        lanelet2.io.write(osm_file, lanelet_map, self.projector)

    def warmup(self):
        '''
        This function is used to warm up the Lanelet2 library by forcing all lanelet centerlines to be cached.
        Avoids lazy-loading delays during routing.
        '''
        for lanelet in self.lanelet_layer:
            _ = lanelet.centerline