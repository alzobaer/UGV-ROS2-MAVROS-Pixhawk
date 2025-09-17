# ROS 2 + MAVROS Setup on Raspberry Pi 5 (Ubuntu 24.04)

## 📌 Overview
This repository documents the process of installing and configuring **ROS 2 Jazzy Jalisco** and **MAVROS** on a **Raspberry Pi 5** running **Ubuntu 24.04 (Noble)**.  
It also includes a custom launch package `ugv_mavros` for connecting to a **Pixhawk (CubePilot CubeOrange+)** flight controller over USB.

---
## 🖼️ System Architecture

![ROS2 MAVROS Pixhawk Diagram](ros2_mavros_pixhawk.png)


---

## 🛠️ Requirements
### Hardware
- Raspberry Pi 5 (4–8 GB recommended)
- 32 GB+ microSD card
- CubePilot CubeOrange+ (Pixhawk flight controller)
- USB cable (Pixhawk ↔ Raspberry Pi)
- Optional: GPS, battery, RC radio

### Software
- Ubuntu 24.04 ARM64
- ROS 2 Jazzy Jalisco
- MAVROS (ROS 2 branch)

---

## 🚀 Installation

### 1. Install ROS 2 Jazzy
```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Create a fresh ROS 2 Workspace for MAVROS
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone MAVROS (ROS 2 branch)
```bash
git clone -b ros2 https://github.com/mavlink/mavros.git
```

### 4. Create a custom wrapper package `ugv_mavros`
```bash
cd ~/ros2_ws/src
ros2 pkg create ugv_mavros --build-type ament_python
```

### 5. Add launch file `ugv_mavros/launch/ugv_mavros.launch.py`:
```bash
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')

    mavros_share = get_package_share_directory('mavros')
    apm_config = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
    apm_plugins = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='Serial port for Pixhawk'
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            # name='mavros',
            output='screen',
            parameters=[
                apm_config,
                apm_plugins,
                {'fcu_url': fcu_url},
                {'gcs_url': ''},
                {'target_system_id': 1},
                {'target_component_id': 1}
            ]
        )
    ])
```

### 6. Ensure `setup.cfg` or `~/ros2_ws/src/ugv_mavros/setup.cfg` has-
```bash
[develop]
script_dir=$base/lib/ugv_mavros
[install]
install_scripts=$base/lib/ugv_mavros
```

### 7. Also ensure `setup.py` or `~/ros2_ws/src/ugv_mavros/setup.py` has-
```bash
from setuptools import setup, find_packages

package_name = 'ugv_mavros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ugv_mavros']),
        ('share/' + package_name + '/launch', ['launch/ugv_mavros.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zobaer',
    maintainer_email='you@example.com',
    description='UGV wrapper for MAVROS (ROS 2)',
    license='MIT',
    entry_points={'console_scripts': []},
)
```

### 8. Prepare package files
```bash
echo "ugv_mavros" > ~/ros2_ws/src/ugv_mavros/resource/ugv_mavros
touch ~/ros2_ws/src/ugv_mavros/ugv_mavros/__init__.py
```

### 9. Install dependencies and build
```bash
cd ~/ros2_ws
rosdep update
rosdep install --rosdistro jazzy --from-paths src -y
colcon build --symlink-install
```

Add sourcing to your ~/.bashrc (once):
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---
## 🔌 Connecting Pixhawk
Check device:
```bash
ls -l /dev/serial/by-id
```

Example output: You will see something like
```bash
usb-CubePilot_CubeOrange+_...-if00 -> ../../ttyACM0
usb-CubePilot_CubeOrange+_...-if02 -> ../../ttyACM1
```
Fix permission:
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

---
## ▶️ Running MAVROS
Launch MAVROS with Pixhawk:
```bash
ros2 launch ugv_mavros ugv_mavros.launch.py \
    fcu_url:=serial:///dev/serial/by-id/<device>-if00:115200
# or 
ros2 launch ugv_mavros ugv_mavros.launch.py
```

Successful connection shows:
```arduino
Got HEARTBEAT, connected. FCU: ArduPilot
```

---
## ✅ Testing
ROS 2 topics
```bash
ros2 topic list
ros2 topic echo /mavros/state
ros2 topic echo /mavros/imu/data
```