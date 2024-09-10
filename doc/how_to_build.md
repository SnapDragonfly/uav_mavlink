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