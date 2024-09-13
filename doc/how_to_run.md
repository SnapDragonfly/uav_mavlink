# How to run the program?

Open a terminal and run roscore.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscore
```

Open another terminal and run uav_uav_bridge_mavlink node.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun uav_uav_bridge uav_uav_bridge_mavlink
```