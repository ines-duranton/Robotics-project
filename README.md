# SAFE_ROS2 — ECE346 Racecar Project

ECE346 ROS 2 Foxy development environment running in Docker.

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

### 3. Create a private fork

**If you've never used git before, we recommend this introductory [tutorial](https://www.atlassian.com/git/tutorials).**

1. In the upper-right corner of any page on [GitHub](https://github.com/), select '+', then click **New repository**.

2. Type `ECE346_GroupXX` as the name for your repository, add a README file, and an optional description.

3. Choose **Private** as your repository visibility.

4. Click **Create repository**.

5. In your terminal, clone the course repository:
    ```bash
    git clone https://github.com/SafeRoboticsLab/ECE346.git
    ```

6. From inside the cloned directory, rename the original `ECE346` GitHub repo to `upstream` (default is `origin`), which you'll use to fetch future lab assignments and updates.
    ```bash
    cd ECE346
    git remote rename origin upstream
    git remote set-url --push upstream DISABLE
    ```

7. Add your new private repository as a new remote named `origin`. To locate your private repo's URL, navigate to its main page on GitHub, select the green `<> Code` icon, select SSH, and copy this URL to your clipboard.
    ```bash
    git remote add origin <URL of your private repo>
    ```

8. Configure your Git identity. **Note**: run these commands inside your ECE346 directory.
    ```bash
    git config --global user.email "your_email@example.com"
    git config --global user.name "Your Name"
    ```

9. Push the `SP2026` branch to your private repository, which has now become a private fork of `ECE346`.
    ```bash
    git push -u origin SP2026
    ```
    If your terminal says "The authenticity of host 'github.com'... continue connecting?" type "yes".

10. Add all course AIs as [collaborators](https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-access-to-your-personal-repositories/inviting-collaborators-to-a-personal-repository) to your private fork: navigate to your private repository's **Settings** > **Collaborators and Teams** under **Access** > **Add People** > CalvinTAVN

### 4. Build the Docker image

```bash
cd ECE346
docker compose build
```

The first build takes a few minutes (downloads ROS 2 Foxy image + installs all dependencies).

### 5. Start the container

```bash
./start.sh
```

This starts the container, and opens a shell inside it.

### 6. Build and run (inside the container)

```bash
colcon build --symlink-install
source install/setup.bash
```

From here, go to the individual lab README in `src/racecar_ece346/ece346/` for lab-specific instructions.

---

## FAQ

### Day-to-Day Usage

```bash
./start.sh          # start container + open shell
./start.sh build    # rebuild image (after Dockerfile/requirements changes)
./start.sh down     # stop the container
```

To open additional terminal windows into the same container:

```bash
sudo docker compose exec ros bash
```

### When do I need to rebuild?

| Change | Action |
|--------|--------|
| Edit `.py` files | Nothing — changes are live instantly |
| Edit `.yaml` config files | Nothing — changes are live instantly |
| Change `.msg`, `.srv`, or `CMakeLists.txt` | Run `colcon build` inside the container |
| Change `Dockerfile` or `requirements.txt` | Run `./start.sh build` |

### Project Structure

```
ECE346/
├── src/
│   ├── racecar_msgs/        # Custom messages (ServoMsg, OdometryArray, SetArray)
│   ├── racecar_routing/     # Lanelet2 map routing + services
│   ├── racecar_interface/   # Simulator, traffic sim, visualization
│   └── racecar_ece346/      # Lab code (Lab0)
├── HOST_setup/linux/       
├── docker/                  # Entrypoint script
├── Dockerfile
├── docker-compose.yml
├── docker-compose.nvidia.yml
├── requirements.txt
└── start.sh                 # launch Container
```

### Pushing your lab solutions

When working on labs and making changes to your code, push to your private repo:
```bash
git add .
git commit -m "Completed Lab X"
git push origin SP2026
```

### Pulling future updates from the course repo

When new labs or updates are released:
```bash
# Make sure your changes are committed first
git add .
git commit -m "Save current work"

# Pull updates from the course repo
git pull upstream SP2026
```

If you encounter merge conflicts, this [tutorial](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts) can help. If you're unsure about merging, you can create a temporary branch first:
```bash
git checkout -b temp
git pull upstream SP2026
# Inspect changes, then merge into your main branch
git checkout SP2026
git merge temp
git branch --delete temp
```


