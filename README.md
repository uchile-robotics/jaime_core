# Bender core


## Table of contents

- [Jaime core](#jaime-core)
  - [Table of contents](#table-of-contents)
  - [Overview](#overview)
  - [Structure](#structure)


## Overview

This repository contains all the core ROS packages that make up the foundational layer of the Jaime robot system. These packages provide low-level functionalities (e.g., motion control, sensors, robot description) that are expected to be always running, either in simulation or on the real robot. Lab members that contribute to this repository should ensure that new code adheres to the structural and modular conventions detailed below.
<!-- This repository contain all ROS packages that make up the core of Bender. These correspond to low-level functionalities that are expected to be always available for the users. To facilitate the use of these packages, it is recommended that new code that is added comply to the following structure. -->

## Structure

- `bender_base`: Launch files and parameters for Jaime’s mobile base (kobuki).
<!--`bender_calibrate`: Tools and configurations for calibrating Bender’s cameras.-->
- `bender_description`: Robot description (URDF/XACRO) and TF tree publisher for the real robot, including 3D meshes.
- `bender_joy`: Joystick control interfaces for teleoperating Bender.
- `bender_sensors`: Drivers and interfaces for onboard sensors (RGB cameras, depth sensors, LIDAR, etc.).
- `bender_sim`: Simulation configurations and assets for running Bender in Gazebo or other environments.
Every package should follow this general layout:

```bash
jaime_<component>/
├── config/ # YAML configuration files
├── launch/ # Launch files
├── src/ # Source code
├── include/ # Header files (if C++ is used)
├── scripts/ # Python scripts
├── msg/ # Custom message definitions (if any)
├── srv/ # Custom service definitions (if any)
├── CMakeLists.txt
├── package.xml
```

