# Lab 0 Truck - Running Pure Pursuit on the Real Truck

Now that you have completed Lab 0 in simulation, it is time to deploy your pure pursuit controller on the real truck. This guide walks you through the hardware, setup, and step-by-step instructions for running your code on the physical platform.

## The Truck

![The Truck](assets/truck.jpeg)

Our truck platform is a **Traxxas Slash 4x4 VXL** 1/10-scale RC truck, modified for autonomous operation. It is equipped with:

- **NVIDIA Jetson** — The onboard computer that runs the motor controller (VESC), joystick teleoperation, and the ackermann mux. It also runs AprilTag SLAM for localization.
- **VESC Motor Controller** — Converts speed and steering commands into motor signals.
- **ZED 2 Camera** — Used by the SLAM system for visual localization via AprilTags.
- **PS4 Controller** — Used for manual remote control and safety overrides.
- **Host Computer** — Your laptop, connected to the truck over WiFi. This is where you run your planning/control algorithms (pure pursuit, ILQR) and visualization (RViz).

The truck uses a two-machine architecture: the **Jetson** handles low-level motor control and localization, while the **host computer** handles high-level planning and visualization. They communicate over ROS 2 using CycloneDDS.

## Batteries

![Battery Compartment](assets/battery.jpeg)

The truck is powered by a **single LiPo battery** that powers both the drive motor and the NVIDIA Jetson. The battery slides into the chassis compartment. There is a **power switch** on the truck that controls power to the Jetson — flip it on to boot the Jetson, and flip it off when you are done.

Make sure the battery is fully charged before use and **never leave it unattended while charging**. Always disconnect the battery when not in use.

## Source Code

The source code for `lab0_truck` is **already provided** — you do not need to fill in any TODOs for the truck variant. It contains a complete pure pursuit controller adapted for the real truck hardware (publishing `AckermannDriveStamped` commands to the VESC instead of `ServoMsg`, and including safety gating via the PS4 controller). You should have already completed Lab 0 in simulation before running on the truck.

## Prerequisites

Before running on the truck, you should have:

1. Completed **Lab 0 in simulation** — your `pure_pursuit.py` should have all TODOs filled in (Tasks 1-5).
2. **Pulled the latest code from upstream** and rebuilt the Docker image:
   ```bash
   git pull upstream SP2026
   ./start.sh build
   ```
   If git asks you to specify how to reconcile divergent branches, run `git config pull.rebase false` first. If you get a merge conflict on a file you modified (e.g. `pure_pursuit.py`), resolve it by keeping your version:
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

### Step 3: Start CycloneDDS + Pure Pursuit on the Host

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

Once confirmed, launch the pure pursuit controller in the same terminal:

```bash
ros2 launch racecar_ece346 lab0_truck_launch.py
```

This starts:
- Your pure pursuit controller node
- The visualization node (RViz + routing)

RViz should open and display the truck's position on the map.

### Driving the Truck

The truck uses a **safety gating system** with the PS4 controller:

- **L2 (held)** — Enables manual remote control. Use the left joystick to drive.
- **R1 (held)** — Enables autonomous control. The pure pursuit controller will send commands to the truck.
- **No buttons pressed** — The truck will not move. This is the safe default state.

To test autonomous driving:
1. In RViz, click **2D Goal Pose** on the top toolbar and click a point on the map.
2. Hold **R1** on the PS4 controller to enable autonomous mode.
3. The truck should drive toward the goal using your pure pursuit controller.
4. Release **R1** at any time to stop autonomous control.
5. Hold **L2** to take over manual control at any time (manual always overrides autonomous).

## Troubleshooting

- **`ros2 topic list` shows no truck topics on host**: Make sure both machines are using the same `DOMAIN_ID` and that the CycloneDDS setup script was sourced in the same terminal.
- **SLAM pose not publishing**: Check that `start_slam.sh` completed successfully. The ZED camera needs to see AprilTags to localize.
- **Truck not responding to PS4 controller**: Make sure the PS4 is paired via Bluetooth and the F1Tenth stack is running. Check that the battery is connected and the power switch is on.
- **Truck moves but steering/speed is wrong**: You may need to tune parameters in `lab0_truck_real_launch.yaml` (e.g., `max_vel`, `max_steer`, `throttle_gain`).
