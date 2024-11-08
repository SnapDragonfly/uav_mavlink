
#define PACKAGE_NAME   "uav_bridge"
#define NODE_NAME      "uav_bridge_camera"
#define PACKAGE_CONFIG "config_camera.yaml"

//#define CAMERA_CODE_DEBUG

#define CAMERA_DEFAULT_IMAGE_TOPIC   "/tmp/uav_bridge/cam0/image_raw"
#define MAVLINK_DEFAULT_IMU_TOPIC    "/tmp/uav_bridge/imu"
#define CAMERA_DEFAULT_TIME_TOPIC    "/tmp/uav_bridge/cam0/time"

#define CAMERA_DEFAULT_IMAGE_SOURCE  "csi://0"
#define CAMERA_DEFAULT_IMAGE_DECODER "--input-decoder=cpu"
#define CAMERA_DEFAULT_IMAGE_CODEC   "--input-codec=h264"

#define CAMERA_ARGV_LEN           32
#define CAMERA_ARGC_LEN           4

#define CAMERA_LATENCY_MS         80


