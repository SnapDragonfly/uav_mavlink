# How to run the program?

# Run program in test environment

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch uav_bridge run_test.launch
```

# Run programs separately

Open terminal 1 and run roscore.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscore
```

Open terminal 2 and run uav_uav_bridge_mavlink node.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun uav_uav_bridge uav_uav_bridge_mavlink
```

Open terminal 3 and run uav_uav_bridge_camera node.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun uav_bridge uav_bridge_camera --help
usage: video-viewer [--help] input_URI [output_URI]

View/output a video or image stream.
See below for additional arguments that may not be shown above.

positional arguments:
    input_URI       resource URI of input stream  (see videoSource below)
    output_URI      resource URI of output stream (see videoOutput below)

videoSource arguments:
    input                resource URI of the input stream, for example:
                             * /dev/video0               (V4L2 camera #0)
                             * csi://0                   (MIPI CSI camera #0)
                             * rtp://@:1234              (RTP stream)
                             * rtsp://user:pass@ip:1234  (RTSP stream)
                             * webrtc://@:1234/my_stream (WebRTC stream)
                             * file://my_image.jpg       (image file)
                             * file://my_video.mp4       (video file)
                             * file://my_directory/      (directory of images)
  --input-width=WIDTH    explicitly request a width of the stream (optional)
  --input-height=HEIGHT  explicitly request a height of the stream (optional)
  --input-rate=RATE      explicitly request a framerate of the stream (optional)
  --input-save=FILE      path to video file for saving the input stream to disk
  --input-codec=CODEC    RTP requires the codec to be set, one of these:
                             * h264, h265
                             * vp8, vp9
                             * mpeg2, mpeg4
                             * mjpeg
  --input-decoder=TYPE   the decoder engine to use, one of these:
                             * cpu
                             * omx  (aarch64/JetPack4 only)
                             * v4l2 (aarch64/JetPack5 only)
  --input-flip=FLIP      flip method to apply to input:
                             * none (default)
                             * counterclockwise
                             * rotate-180
                             * clockwise
                             * horizontal
                             * vertical
                             * upper-right-diagonal
                             * upper-left-diagonal
  --input-loop=LOOP      for file-based inputs, the number of loops to run:
                             * -1 = loop forever
                             *  0 = don't loop (default)
                             * >0 = set number of loops

videoOutput arguments:
    output               resource URI of the output stream, for example:
                             * file://my_image.jpg       (image file)
                             * file://my_video.mp4       (video file)
                             * file://my_directory/      (directory of images)
                             * rtp://<remote-ip>:1234    (RTP stream)
                             * rtsp://@:8554/my_stream   (RTSP stream)
                             * webrtc://@:1234/my_stream (WebRTC stream)
                             * display://0               (OpenGL window)
  --output-codec=CODEC   desired codec for compressed output streams:
                            * h264 (default), h265
                            * vp8, vp9
                            * mpeg2, mpeg4
                            * mjpeg
  --output-encoder=TYPE  the encoder engine to use, one of these:
                            * cpu
                            * omx  (aarch64/JetPack4 only)
                            * v4l2 (aarch64/JetPack5 only)
  --output-save=FILE     path to a video file for saving the compressed stream
                         to disk, in addition to the primary output above
  --bitrate=BITRATE      desired target VBR bitrate for compressed streams,
                         in bits per second. The default is 4000000 (4 Mbps)
  --stun-server=URL      WebRTC connection STUN server (set to 'disabled' for LAN)
  --headless             don't create a default OpenGL GUI window

logging arguments:
  --log-file=FILE        output destination file (default is stdout)
  --log-level=LEVEL      message output threshold, one of the following:
                             * silent
                             * error
                             * warning
                             * success
                             * info
                             * verbose (default)
                             * debug
  --verbose              enable verbose logging (same as --log-level=verbose)
  --debug                enable debug logging   (same as --log-level=debug)
```

typical csi://0 camera

```
$ rosrun uav_bridge uav_bridge_camera --input-flip=rotate-180 --input-width=752 --input-height=480 --input-rate=5 csi://0
```
