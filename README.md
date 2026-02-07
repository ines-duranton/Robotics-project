# SAFE_ROS2 — ECE346 Racecar Project

ROS 2 Foxy development environment running in Docker. Works on any x86_64 Ubuntu laptop regardless of GPU (AMD, Intel, or NVIDIA).

## Prerequisites

- Ubuntu 20.04+ (any version)
- Git

## Fresh Laptop Setup

### 1. Install Docker

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo usermod -aG docker $USER
newgrp docker
```

Verify: `docker --version`

### 2. Install NVIDIA Container Toolkit

Most lab machines have NVIDIA GPUs. If you have AMD/Intel, skip this step.

```bash
# Add NVIDIA container toolkit repo
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify: `nvidia-smi` should show your GPU.

### 3. Clone and build

```bash
git clone <repo-url>
cd SAFE_ROS2
docker compose build
```

The first build takes a few minutes (downloads ROS 2 Foxy image + installs all dependencies).

### 4. Start the container

```bash
./start.sh
```

This auto-detects your GPU, starts the container, and opens a shell inside it.

### 5. Build and run (inside the container)

```bash
colcon build --symlink-install
source install/setup.bash
```

Launch a lab:

```bash
# Lab 1 — Pure Pursuit Controller
ros2 launch racecar_ece346 lab1_simulation_launch.py

# Lab 2 — ILQR Trajectory Planning
ros2 launch racecar_ece346 lab2_simulation_launch.py

# Lab 3 — FRS + Obstacle Avoidance
ros2 launch racecar_ece346 Lab3_simulation_launch.py
```

## Day-to-Day Usage

```bash
./start.sh          # start container + open shell
./start.sh build    # rebuild image (after Dockerfile/requirements changes)
./start.sh down     # stop the container
```

To open additional terminal windows into the same container:

```bash
docker compose exec ros bash
```

## When do I need to rebuild?

| Change | Action |
|--------|--------|
| Edit `.py` files | Nothing — changes are live instantly |
| Edit `.yaml` config files | Nothing — changes are live instantly |
| Change `.msg`, `.srv`, or `CMakeLists.txt` | Run `colcon build` inside the container |
| Change `Dockerfile` or `requirements.txt` | Run `./start.sh build` |

## Project Structure

```
SAFE_ROS2/
├── src/
│   ├── racecar_msgs/        # Custom messages (ServoMsg, OdometryArray, SetArray)
│   ├── racecar_routing/     # Lanelet2 map routing + services
│   ├── racecar_interface/   # Simulator, traffic sim, visualization
│   └── racecar_ece346/      # Lab code (Lab1, Lab2, Lab3)
├── HOST_setup/linux/        # pyspline wheel
├── docker/                  # Entrypoint script
├── Dockerfile
├── docker-compose.yml
├── docker-compose.nvidia.yml
├── requirements.txt
└── start.sh                 # Auto-detect GPU and launch
```

## Troubleshooting

**rviz2 shows libGL errors:** The container can't access your GPU. Make sure `/dev/dri` exists on your host (`ls /dev/dri`). For NVIDIA, ensure `nvidia-container-toolkit` is installed.

**`xhost: command not found`:** Install with `sudo apt-get install x11-xserver-utils`.

**`colcon build` fails with "ament_cmake not found":** You forgot to source ROS 2. Run `source /opt/ros/foxy/setup.bash` first. This happens automatically on new shells after an image rebuild.

**Permission denied on Docker commands:** Run `sudo usermod -aG docker $USER` then log out and back in.
