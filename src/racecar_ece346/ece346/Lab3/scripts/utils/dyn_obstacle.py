from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from racecar_ece346.srv import GetFRS  
import numpy as np

def frs_to_msg(frs_respond: GetFRS.Response) -> MarkerArray:
    """
    Convert GetFRSResponse to MarkerArray (visualization-safe version).
    Ensures type correctness for ROS2 geometry_msgs/Point requirements.
    """
    marker_array = MarkerArray()

    if frs_respond.frs is None:
        return marker_array

    for i, frs in enumerate(frs_respond.frs):  # list of SetArray
        for t, frs_t in enumerate(frs.set_list):  # list of polygon sets

            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = f"frs_{i}"
            marker.id = t
            marker.action = Marker.ADD
            marker.type = Marker.LINE_STRIP

            marker.color.r = 204.0 / 255.0
            marker.color.g = 102.0 / 255.0
            marker.color.b = 0.0 / 255.0
            marker.color.a = 0.5

            marker.scale.x = 0.01

            # --- FIX: convert all points explicitly to geometry_msgs/Point ---
            marker.points = []
            for p_in in frs_t.points:  
                p = Point()
                p.x = float(p_in.x)
                p.y = float(p_in.y)
                p.z = float(getattr(p_in, 'z', 0.0))  # handle Point32 case
                marker.points.append(p)

            # Close polygon if needed
            if len(marker.points) > 0:
                marker.points.append(marker.points[0])

            marker.lifetime = Duration(sec=0, nanosec=200_000_000)

            marker_array.markers.append(marker)

    return marker_array



def frs_to_obstacle(frs_respond: GetFRS.Response) -> list:
    """
    Convert GetFRSResponse to obstacle vertex lists usable by ILQR.
    """
    
    obstacles_list = []

    # frs_respond.frs is an array of SetArray
    for frs in frs_respond.frs:
        vertices_list = []

        # frs.set_list is a list of Polygons
        for frs_t in frs.set_list:
            polygon = []
            for point in frs_t.points:
                polygon.append([point.x, point.y])
            vertices_list.append(np.array(polygon))

        obstacles_list.append(vertices_list)

    return obstacles_list