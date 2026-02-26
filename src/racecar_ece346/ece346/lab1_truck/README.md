# Lab 1 Truck - Running ILQR Trajectory Planning on the Real Truck

Now that you have completed Lab 1 in simulation, it is time to deploy your ILQR trajectory planner on the real truck. If you have not already read the [Lab 0 Truck README](../lab0_truck/README.md), please do so first — it covers the truck hardware, battery setup, and the general workflow for running code on the truck.

## Source Code

The source code for `lab1_truck` contains the same TODO stubs as Lab 1 in simulation — you must complete the following before running on the truck:

- **`ilqr.py`** — TODO 1b (backward pass), TODO 1c (forward pass), and TODO 1 (ILQR main loop in `plan`)
- **`traj_planner.py`** — TODO Task 2 (`compute_control`) and TODO Task 3 (`receding_horizon_planning_thread`)

The truck variant includes the following adaptations for real hardware (already provided):
- Publishes `AckermannDriveStamped` commands to the VESC instead of `ServoMsg`
- Safety gating via the PS4 controller (`/autonomous_lock` topic)
- Reads odometry from `/SLAM/Pose` instead of the simulator

## Prerequisites

Before running on the truck, you should have:

1. Completed **Lab 1 in simulation** — all TODOs in `ilqr.py` and `traj_planner.py` should be filled in and working.
2. **Pulled the latest code from upstream** and rebuilt the Docker image:
   ```bash
   git pull upstream SP2026
   ./start.sh build
   ```
   If git asks you to specify how to reconcile divergent branches, run `git config pull.rebase false` first. If you get a merge conflict on a file you modified, resolve it by keeping your version:
   ```bash
   git checkout --ours <path-to-conflicting-file>
   git add <path-to-conflicting-file>
   git commit
   ```
3. The truck powered on — battery connected, power switch flipped on, Jetson booted.
4. Your host computer connected to the **same WiFi network** as the truck.
5. Know your **host's IP address** (use `hostname -I` to find it). The truck's IP is `192.168.1.2XX` where `XX` is your Jetson's ID number.

## Running on the Truck

You will need **3 terminals**: 2 on the truck (via SSH) and 1 on your host computer.

### Step 1: Start SLAM on the Truck

Open a terminal and SSH into the truck. The truck's IP address is `192.168.1.2XX` where `XX` is your Jetson's ID number. The password is always `nvidia`.

```bash
ssh nvidia@192.168.1.2XX
```

Navigate to the truck workspace and start SLAM:

```bash
cd ~/ece346_truck
./start_slam.sh
```

This script will:
1. Launch AprilTag SLAM (ZED camera + SLAM node)
2. Wait for the SLAM service to become available, then start it

Wait until you see `SLAM started.` before proceeding.

### Step 2: Start CycloneDDS + F1Tenth Stack on the Truck

Open a **second terminal** and SSH into the truck:

```bash
ssh nvidia@192.168.1.2XX
```

Source CycloneDDS and launch the F1Tenth stack:

```bash
cd ~/ece346_truck
source setup_cyclone.sh <HOST_IP> <DOMAIN_ID>
ros2 launch f1tenth_stack bringup_launch.py
```

Replace `<HOST_IP>` with your host computer's IP address and `<DOMAIN_ID>` with your group's unique domain ID (0-101). This starts the joystick driver, VESC motor controller, ackermann mux, and the ROS 2 ZMQ bridge.

**Test remote control**: At this point, you should be able to drive the truck with the PS4 controller by holding **L2** and using the left joystick.

### Step 3: Start CycloneDDS + ILQR Planner on the Host

On your **host computer**, open a terminal into the Docker container:

```bash
./start.sh
```

Make sure you have pulled the latest code and rebuilt inside the container:

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Then source CycloneDDS:

```bash
source /ros2_ws/setup_cyclone.sh 192.168.1.2XX <DOMAIN_ID>
```

Use the **same `<DOMAIN_ID>`** as the truck. Replace `192.168.1.2XX` with the truck's IP address.

Before launching, verify that the truck's topics are visible:

```bash
ros2 topic list
```

You should see topics like `/SLAM/Pose`, `/ackermann_drive`, etc. You can also verify SLAM data is flowing:

```bash
ros2 topic hz /SLAM/Pose
```

Once confirmed, launch the ILQR trajectory planner in the same terminal:

```bash
ros2 launch racecar_ece346 lab1_truck_launch.py
```

This starts the trajectory planning node. Wait for the ILQR warm-up to complete — you should see `ILQR warm up finished.` in the terminal.

### Driving the Truck

The truck uses a **safety gating system** with the PS4 controller:

- **L2 (held)** — Enables manual remote control. Use the left joystick to drive.
- **R1 (held)** — Enables autonomous control. The ILQR planner will send commands to the truck.
- **No buttons pressed** — The truck will not move. This is the safe default state.

To test autonomous driving:
1. In RViz, click **2D Goal Pose** on the top toolbar and click a point on the map. The routing node will compute a path.
2. Hold **R1** on the PS4 controller to enable autonomous mode.
3. The truck should follow the planned trajectory using your ILQR controller.
4. Release **R1** at any time to stop autonomous control.
5. Hold **L2** to take over manual control at any time (manual always overrides autonomous).

## Tuning

You can tune ILQR and planner parameters at runtime using **Dynamic Reconfigure** in RQT, or by editing the config files:

- **`lab1_truck_real_launch.yaml`** — Planner parameters (e.g., `replan_dt`, `truck_latency`)
- **`truck_ilqr.yaml`** — ILQR parameters (e.g., cost weights, horizon length, dynamics parameters)

The `truck_latency` parameter compensates for communication delay between the host and the truck. You may need to adjust this for best performance.

## Troubleshooting

- **`ros2 topic list` shows no truck topics on host**: Make sure both machines are using the same `DOMAIN_ID` and that the CycloneDDS setup script was sourced in the same terminal.
- **SLAM pose not publishing**: Check that `start_slam.sh` completed successfully. The ZED camera needs to see AprilTags to localize.
- **Truck not responding to PS4 controller**: Make sure the PS4 is paired via Bluetooth and the F1Tenth stack is running. Check that the battery is connected and the power switch is on.
- **ILQR warm-up takes a long time**: This is normal on the first run — Jax needs to compile the functions. Subsequent runs will be faster.
- **Truck oscillates or behaves erratically**: Try increasing `truck_latency` to compensate for communication delay, or reduce the target speed in the ILQR cost weights.
