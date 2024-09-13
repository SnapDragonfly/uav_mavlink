# How to build?

Upgrade and install depended libraries.

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install libyaml-cpp-dev
$ sudo apt-get install libeigen3-dev
```

Clone repo and build app.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/SnapDragonfly/uav_mavlink.git
$ cd uav_mavlink
$ git submodule init
$ git submodule update
$ cd ../../
$ catkin_make --pkg uav_bridge
```