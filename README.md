# uav_mavlink

uav_mavlink is used for bridging the message between ardupilot and VINS-Mono/ego-Planner.

- hardware diagram
```
imx415 --CSI--> ssc338q(OpenIPC) <--USB--> rtl8812au
                       ^                       ^
                       |                       |
                      UART                    WiFi
                       |                       |
                       v                       v
                H743 FC(Ardupilot)   JetsonOrin(VINS Mono+ego Planner)
```

- software diagram
```
H743 FC(Ardupilot)  <-- uav_mavlink --> ego Planner
                           ^
                           |
                           v
                        VINS Mono
```

*Note: Diagram is from [Ardupilot FollowMe Test Platform Build](https://blog.csdn.net/lida2003/article/details/141649074).*

# How to build?

Upgrade and install dev libraries.

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install libyaml-cpp-dev
$ sudo apt-get install libeigen3-dev
```

clone repo and build app.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/SnapDragonfly/uav_mavlink.git
$ cd ../
$ catkin_make --pkg uav_mavlink
```

# How to configure?

TBD.


# How to run the program?

Open a terminal and run roscore.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscore
```

Open another terminal and run uav_mavlink app.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun uav_mavlink uav_mavlink
```

