# METR4202

This is the source code for the 2022 METR4202 team project, hosted
[on GitHub](https://github.com/Ethan1023/METR4202). Clone it with
```bash
$ git clone https://github.com/Ethan1023/METR4202.git
```

Package dependencies and RPi setup are all listed in a
[very nice repository](https://github.com/UQ-METR4202/METR4202_S2-2022_Resources/blob/main/RPi4_Setup.md).

## Setup and Launch

After cloning this project, a few steps are necessary before it will run.
First, source ROS Noetic using:
```bash
$ source /opt/ros/noetic/setup.bash
```

### Camera setup

Running the Ximea camera requires a reasonably lengthy setup and calibration,
as documented in [UQ-METR4202/metr4202_ximea_ros](https://github.com/UQ-METR4202/metr4202_ximea_ros).

Assuming this has been done, and the correct camera IDs are used specified in
the `ximea_ros` and `ximea_colour`, the only necessary modification is adding
the following line to `ximea_ros/launch/ximea_aruco.launch`:
```xml
        <arg name="fiducial_len" value="0.028"/>
```

Add this line under the last existing argument within the `<include>` tags. The
value of this argument should be set to the side length (in meters) of the ArUco
tags that will be used.

Finally, if not done already, run the following command after each boot to
disable the USB memory limits:
```bash
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

As a final note, the accuracy of the camera is highly dependent on all of zoom,
focus and aperture, as well as lighting conditions. Assuming the zoom and focus
are reasonably well set and the lighting conditions are sufficient, if the
camera is having trouble detecting ArUco tags, this may suggest the aperture is
too large.

Beware that there is a trade-off here, as reducing the aperature too much may
result in inconsistent colour detection. This is best tuned experimentally by
trial and error in the environment where it will be used.

### Make and launch

Make the project by changing into the catkin_ws directory (or moving
the entire project into `~/catkin_ws/src` if not there already).
```bash
$ cd ~/catkin_ws && catkin_make && source ./devel/setup.bash
```

Start the ROS master:
```bash
$ roscore
```

Now in a new terminal, the project can be launched using:
```bash
$ roslaunch metr4202 project.launch
```

The nodes will start printing to the terminal, and two camera views will pop up.
Change the camera view to colour by activating the first camera view and hit
spacebar. The robot is now ready to sort ~~and throw~~ your luggage!

## ROS Details

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

## Other
Dynamixel config for https://github.com/UQ-METR4202/dynamixel_interface can be found in configs/
Ximea camera calibration can be found in configs/
