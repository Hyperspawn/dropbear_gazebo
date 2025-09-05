# Dropbear Gazebo - Robot Simulation

A workspace containing a modular humanoid robot Dropbear with full Gazebo simulation support and control system.

## 🚀 Features

- **Modular Design**: Separate packages for each robot component (head, torso, arms, legs, pelvis, battery)
- **Gazebo Simulation**: Full 3D simulation with physics and visualization
- **ROS 2 Control**: Integrated control system with multiple controllers for different robot parts
- **Stewart Platform Head**: Advanced 6-DOF head control system
- **Complete URDF/Xacro**: Detailed robot description with materials and meshes

## 📁 Project Structure

```
dropbear_gazebo/
├── dropbear_granular_urdf/     # Main robot description package
│   ├── launch/                 # Launch files for simulation
│   ├── urdf/                   # Robot description files (Xacro/URDF)
│   ├── meshes/                 # 3D mesh files organized by component
│   ├── config/                 # Controller configurations
│   └── rviz/                   # RViz configuration files
├── battery/                    # Battery component package
├── head/                       # Head component package
├── pelvis/                     # Pelvic girdle component package
├── right_arm/                  # Right arm component package
├── right_leg/                  # Right leg component package
└── torso/                      # Torso component package
```

## 🛠️ Prerequisites

- **ROS 2** (Humble or later recommended)
- **Gazebo** (with ROS 2 integration)
- **colcon** build system
- **xacro** package
- **gazebo_ros2_control** package

### Install Dependencies

```bash
# Install ROS 2 (if not already installed)
# Follow instructions at: https://docs.ros.org/en/humble/Installation.html

# Install required packages
sudo apt update
sudo apt install ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-robot-state-publisher

# Install colcon (if not already installed)
sudo apt install python3-colcon-common-extensions
```

## 🏗️ Building the Workspace

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd dropbear_gazebo
   ```

2. **Build the workspace:**
   ```bash
   colcon build
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## 🎮 Running the Simulation

### Launch Gazebo Simulation

```bash
# Launch the complete robot in Gazebo
ros2 launch dropbear_granular_urdf hyperspawn_dropbear_gazebo.launch.py
```

### Launch RViz Visualization

```bash
# Launch RViz for robot visualization (without Gazebo)
ros2 launch dropbear_granular_urdf hyperspawn_dropbear_display.launch.py
```

## 🤖 Robot Components

### Controllers Available

The robot includes the following controllers:

- **Head**: Stewart platform slider controller (6-DOF)
- **Arms**: 
  - Hand controllers (position control)
  - Elbow controllers (effort control)
- **Legs**:
  - Hip joint controllers
  - Knee controllers
  - Foot controllers
- **Pelvis**: Waist joint controller
- **Battery**: Power management system

### Control Topics

Once the simulation is running, you can control the robot using standard ROS 2 control interfaces:

```bash
# List available controllers
ros2 control list_controllers

# Send commands to specific controllers
ros2 topic pub /stewart_slider_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

## 🔧 Configuration

### Controller Configuration

Controller parameters are defined in `dropbear_granular_urdf/config/controllers.yaml`. You can modify:

- Update rates
- Joint limits
- Control gains
- Controller types

### Robot Description

The main robot description is in `dropbear_granular_urdf/urdf/dropbear.urdf.xacro`. This file includes:

- Component macros for each robot part
- Joint configurations
- Material definitions
- ROS 2 control integration

## 🐛 Troubleshooting

### Common Issues

1. **Gazebo not starting**: Ensure Gazebo is properly installed with ROS 2 integration
2. **Missing meshes**: All mesh files are included in the repository
3. **Controller errors**: Check that `gazebo_ros2_control` is installed
4. **Build errors**: Ensure all ROS 2 dependencies are installed

### Debug Commands

```bash
# Check robot description
ros2 topic echo /robot_description

# View joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers
```

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request


## Dev Support

For technical support and questions:

- [Hyperspawn.co](https://hyperspawn.co)
- [GitHub Issues](https://github.com/hyperspawn/dropbear_isaac/issues)
- [Discord](https://discord.com/invite/tFeqrdJzkS)
- [TG](https://t.me/fractionalrobots)
- [Docs](https://www.hyperspawn.co/docs)
- [Gallery](https://hyperspawn.co/gallery)
- [Buy Robot](https://hyperspawn.co/buy)
- [Web-Simulator](https://hyperspawn.co/sim)
- Contact the maintainer: Priyanshupareek (ppareek85@gmail.com)


---

**Note**: This is a simulation package. For real robot deployment, additional safety considerations and hardware-specific configurations may be required.
