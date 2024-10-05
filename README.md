# uav bridge

- [How to build?](doc/how_to_build.md)
- [How to configure?](doc/how_to_config.md)
- [How to run the program?](doc/how_to_run.md)
- [Hardware diagram for uav mavlink](doc/hardware_diagram.md)
- [Software diagram for uav mavlink](doc/software_diagram.md)

# Design Goal

uav_mavlink is used for bridging messages between ardupilot and VINS-Mono/VINS-Fusion/ego-Planner.
And it oriented to simplify configuration and handles different kind of interfaces.

*The original code comes from: https://github.com/chobitsfan/mavlink-udp-proxy*


[![Watch uav_bridge rviz test](https://img.youtube.com/vi/ewxGkjimbnc/0.jpg)](https://youtu.be/ewxGkjimbnc)

# Features

- [x] Support UART connection with Ardupilot
- [x] Support UDP Client connection with Ardupilot
- [x] Support UDP Server connection from Ardupilot
- [x] Support RTP splitter from uav_mixer
- [ ] Support loopback control to uav_mixer
- [x] Support CSI/RTP video feed
- [ ] IPC1???
- [ ] IPC2???

# ToDo

- clarify IPC1/IPC2 usage
- join test with wfb-ng



