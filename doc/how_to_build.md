# How to build?

- Step 1: Upgrade and install depended libraries.

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install libyaml-cpp-dev
$ sudo apt-get install libeigen3-dev
```

- Step 2: Clone cv-bridge //used for uav_bridge_camera

```
$ sudo apt-get remove ros-noetic-cv-bridge
$ git clone git@github.com:ros-perception/vision_opencv.git
$ cd vision_opencv/
```

- Step 3: Checkout right branch // noetic for jetson orin nano 8GB Linux 35.5

```
$ rosversion -d
noetic
$ git checkout noetic
```

- Step 4: Build and install custom build cv_bridge

```
$ cd cv_bridge/
$ mkdir build
$ cd build
$ cmake ..
$ sudo make
$ sudo make install
```

- Step 5: Clone repo

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/SnapDragonfly/uav_mavlink.git
$ cd uav_mavlink
$ git submodule init
$ git submodule update
```

-  Step 6: Build repo

```
$ cd ../../
$ catkin_make --pkg uav_bridge
```

