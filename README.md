# uav mavlink

- [How to build?](doc/how_to_build.md)
- [How to configure?](doc/how_to_config.md)
- [How to run the program?](doc/how_to_run.md)
- [Hardware diagram for uav mavlink](doc/hardware_diagram.md)

# Design Goal

uav_mavlink is used for bridging the messages between ardupilot and VINS-Mono/VINS-Fusion/ego-Planner.

# Software Diagram

```
Ardupilot  <--UART/UDP --> uav_mavlink <--> ego Planner
                                ^
                                |
                                v
                         VINS Mono/Fusion
```

# Features

1. Support UART connection with Ardupilot
2. Support UDP Client connection with Ardupilot
3. Support UDP Server connection from Ardupilot
4. IPC1???
5. IPC2???



