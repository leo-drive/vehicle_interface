# Leo Vehicle Interface

This package is developed to provide communication between Autoware.Universe and vehicle.

Clone this repository into your workspace (i.e. for Autoware):
```
$ cd ~/projects/autoware/src/vehicle/external
$ git clone --recursive https://github.com/leo-drive/leo_vehicle_interface.git
$ cd ~/projects/autoware/ 
```

Install dependent ROS packages.
```
$ source /opt/ros/humble/setup.bash
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
Build the workspace.
```
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 1. Serial Communication

usb-naming script is used to assign a name to communication port. After go to the configs directory run the command:
```
$ cd configs
$ sudo ./usb-naming.sh
```

Now, check the name of the LLC port name by using:

`$ ls /dev/tty*`

You should see `/dev/ttyLLC`.

### Launch Driver (With Serial Interface)
```
$ ros2 launch leo_vcu_driver leo_vcu_driver.launch.xml interface_mod:=SERIAL
```

## 2. CAN Communication

WIP
### Launch Driver (With CAN Interface)
```
$ ros2 launch leo_vcu_driver leo_vcu_driver.launch.xml interface_mod:=CAN
```