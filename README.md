# SEN771 AGVC — Path Planning Simulation
**Student:** Dexter Leong | s223026243
**Unit:** SEN771 Intelligent Autonomous Robots

---

## After Extracting This Zip

This is a ROS 2 package. To run it, the extracted folder must be placed inside a ROS 2 workspace.

**Step 1 — Place the folder here:**
```
~/ros2_ws/src/sen771_agvc/     ← put this entire folder here
```

If the `ros2_ws` workspace does not exist yet:
```bash
mkdir -p ~/ros2_ws/src
cp -r sen771_agvc ~/ros2_ws/src/
```

**Step 2 — Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select sen771_agvc
```

**Step 3 — Source the workspace:**
```bash
source ~/ros2_ws/install/setup.bash
```

**Step 4 — Run:**
```bash
ros2 launch sen771_agvc sen771.launch.py
```

With a specific obstacle size:
```bash
ros2 launch sen771_agvc sen771.launch.py obs_size:=5.0
```

That's it. Gazebo opens, the robot plans its path and drives autonomously.

---

## System Requirements

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Ionic (Harmonic) |
| Python | 3.12 (system) |

Install ROS 2 + Gazebo if not already installed:
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz
sudo apt install python3-colcon-common-extensions
```

> **Note:** If Miniconda or Anaconda is installed, build with:
> ```bash
> PATH=/usr/bin:/usr/local/bin:$PATH colcon build --packages-select sen771_agvc
> ```

---

## Launch Options

| Argument | Default | Description |
|---|---|---|
| `obs_size:=5.0` | random | Average obstacle size in metres |
| `autostart:=false` | true | Start Gazebo paused (press Play manually) |
| `rviz:=true` | false | Open RViz2 alongside Gazebo |

---

## How It Works

Every launch a Python script generates 15 random red obstacles in safe positions (clear of targets and robot start), writes the Gazebo world file, then launches the simulation. The robot then:

1. Builds a 120×90 occupancy grid with obstacle inflation
2. Solves TSP (brute-force 4! = 24 permutations) for optimal target visit order
3. Plans collision-free paths using A\*, BFS, RRT, and Theta\*
4. Drives autonomously using a Pure Pursuit controller at 50 Hz

---

## Folder Structure

```
sen771_agvc/
├── scripts/
│   └── generate_world.py      ← random obstacle generator (runs at launch)
├── launch/
│   └── sen771.launch.py       ← main launch file
├── sen771_agvc/
│   ├── planner_node.py        ← path planning + robot controller
│   ├── monitor_node.py
│   ├── debug_node.py
│   └── __init__.py
├── worlds/
│   ├── sen771_world_template.sdf  ← base world (field, robot, targets)
│   └── sen771_world.sdf           ← fallback fixed world
├── rviz/
│   └── sen771.rviz
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Stop the Simulation

```bash
pkill -f gz_sim && pkill -f planner && pkill -f parameter_bridge
```

---

## Credits & Acknowledgements

| Resource | Source |
|---|---|
| Husky A200 robot meshes (`base_link.dae`, wheels) | [Clearpath Robotics — clearpath_common](https://github.com/clearpathrobotics/clearpath_common) |
| ROS 2 ↔ Gazebo bridge (`ros_gz_sim`, `ros_gz_bridge`) | [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) |
| ROS 2 Jazzy framework | [ros2/ros2](https://github.com/ros2/ros2) |
| Gazebo Ionic (Harmonic) simulator | [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) |

All path planning algorithms (A\*, BFS, RRT, Theta\*), TSP solver, occupancy grid, Pure Pursuit controller, and random world generator were written from scratch for this assignment.

---

## Fresh Windows Setup (From Scratch)

If you are on a Windows PC with nothing installed yet, follow these steps.

### Step 1 — Install WSL2 + Ubuntu 24.04

Open **PowerShell as Administrator** and run:
```powershell
wsl --install -d Ubuntu-24.04
```
Restart when prompted. Then open **Ubuntu 24.04** from the Start Menu and set a username and password.

### Step 2 — Install ROS 2 Jazzy + Gazebo

Inside the Ubuntu terminal:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz
sudo apt install -y python3-colcon-common-extensions
```

Add ROS 2 to every terminal automatically:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3 — Get the Project Files

Option A — from GitHub:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/DexterLMW/sen771_agvc.git
```

Option B — from the zip file: copy the `sen771_agvc/` folder into Ubuntu via Windows Explorer:
```
\\wsl.localhost\Ubuntu\home\<your-ubuntu-username>\ros2_ws\src\
```

### Step 4 — Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select sen771_agvc
source install/setup.bash
ros2 launch sen771_agvc sen771.launch.py
```

Gazebo opens automatically — WSLg handles the display on Windows 11, no extra setup needed.

### Optional: Better Performance

Create `C:\Users\<YourName>\.wslconfig` on Windows:
```ini
[wsl2]
processors=8
memory=10GB
swap=8GB
```
Then run `wsl --shutdown` in PowerShell and relaunch Ubuntu.
