# METR4202

The source code for this project is hosted [on GitHub](https://github.com/Ethan1023/METR4202). Clone it with
```bash
$ git clone https://github.com/Ethan1023/METR4202.git
```

This project is run by launching
```bash
$ roslaunch metr4202 project.launch
```

### Consists of components of a ROS package, including
* Launch files in launch/
* Python scripts in scripts/
* Custom messages in msg/
* Dependencies in package.xml
* Build information in CMakeLists.txt

### Launch files
* project.launch
* detection.launch
* control.launch

### Current nodes
* state_machine
* joint_controller
* box_transform
* colour_detection
* gripper
