# ROS 2 Package: <Your Package Name>

## Overview

This package provides launch files and nodes for visualizing and interacting with various robot models using RViz and ROS 2. It includes configurations for displaying joint angles and robot states.

## Features

- Launch files for displaying individual and multiple robot models.
- RViz configurations for visualizing robot states and joint angles.
- Python scripts for dynamically generating URDF and RViz configurations.

## Installation

1. **Clone the repository:**

   ```bash
   git clone <repository_url> ~/ros2_ws/src/<your_package_name>
   ```

2. **Build the package:**

   Navigate to your ROS 2 workspace and build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select <your_package_name>
   ```launch/display.launch.py

3. **Source the workspace:**

   After building, source the setup file to overlay this workspace on top of your environment:

   ```bash
   source install/setup.bash
   ```

## Usage

### Launching Individual Robot Display

To launch the display for a specific robot model, use the following command:

```

### Launching All Robot Displays

To launch the display for all robot models, use the following command:

```bash
ros2 launch erob_description all_display.launch.py
```

### Launching All Robot Displays with Joint Angle Display

