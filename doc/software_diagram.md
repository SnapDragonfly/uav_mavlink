# Software Diagram

## Solution A
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

## Solution B
```
  Ardupilot      uav_bridge_mavlink <----------- ego Planner
       ^            |                                   ^
  UART |            | UDP                               |
       v            v                                   |
    uav_mixer <-- wfb-ng --> uav_bridge_splitter --> VINS Mono/Fusion
       ^                 RTP                    topic:imu/img
   RTP |                        
    Camera     
```