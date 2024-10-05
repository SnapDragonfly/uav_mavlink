# hardware diagram

*Note: Diagram is from [Ardupilot FollowMe Test Platform Build](https://blog.csdn.net/lida2003/article/details/141649074).*


## Solution A
```
imx415 --CSI--> ssc338q(OpenIPC) <--USB--> rtl8812au
                       ^                       ^
                       |                       |
                      UART                    WiFi
                       |                       |
                       v                       v
                H743 FC(Ardupilot)   JetsonOrin(VINS Mono+ego Planner)
```

## Solution B
```
imx415 --CSI--> ssc338q(OpenIPC) <--USB--> rtl8812au
                       ^                       ^
                       |                       |
                      UART                    WiFi
                       |                       |
                       v                       v
             H743 FC(Ardupilot) <--UART-->  JetsonOrin(VINS Mono+ego Planner)
```

## Solution C
```
Camera -- CSI --> Linux(SBC) <-- USB --> rtl8812au
                       ^                       ^
                       |                       |
                      UART                    WiFi
                       |                       |
                       v                       v
             H743 FC(Ardupilot)     JetsonOrin(VINS Mono+ego Planner)
```
