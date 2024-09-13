# uav bridge

- [How to build?](doc/how_to_build.md)
- [How to configure?](doc/how_to_config.md)
- [How to run the program?](doc/how_to_run.md)
- [Hardware diagram for uav mavlink](doc/hardware_diagram.md)

# Design Goal

uav_mavlink is used for bridging messages between ardupilot and VINS-Mono/VINS-Fusion/ego-Planner.
And it oriented to simplify configuration and handles different kind of interfaces.

*The original code comes from: https://github.com/chobitsfan/mavlink-udp-proxy*

# Software Diagram

```
Ardupilot  <--UART/UDP --> uav_bridge_mavlink <--> ego Planner
                                ^
                                |
                                v
                         VINS Mono/Fusion
                                ^
                                |
Camera  <-- CSI/RTP -->  uav_bridge_camera
```

# Features

- [x] Support UART connection with Ardupilot
- [x] Support UDP Client connection with Ardupilot
- [x] Support UDP Server connection from Ardupilot
- [ ] IPC1???
- [ ] IPC2???
- [ ] Support CSI/RTP video feed



