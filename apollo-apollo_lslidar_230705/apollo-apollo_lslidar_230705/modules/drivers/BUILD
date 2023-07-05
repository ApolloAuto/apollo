load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "drivers",
    data = [
        ":cyberfile.xml",
        ":drivers.BUILD",
    ],
    deps = [
        ":pb_drivers",
        ":pb_hdrs",
        "//modules/drivers/canbus:install",
        "//modules/drivers/camera:install",
        "//modules/drivers/gnss:install",
        "//modules/drivers/lidar:install",
        "//modules/drivers/microphone:install",
        "//modules/drivers/radar:install",
        "//modules/drivers/smartereye:install",
        "//modules/drivers/tools/image_decompress:install",
        "//modules/drivers/video:install",
        "//modules/drivers/canbus/can_client:install",
        "//modules/drivers/canbus/common:install",
        "//modules/drivers/gnss/test:install",
        "//modules/drivers/lidar/hesai:install",
        "//modules/drivers/lidar/robosense:install",
        "//modules/drivers/lidar/velodyne/compensator:install",
        "//modules/drivers/lidar/velodyne/driver:install",
        "//modules/drivers/lidar/velodyne/fusion:install",
        "//modules/drivers/lidar/velodyne/parser:install",
        "//modules/drivers/lidar/velodyne:install",
        "//modules/drivers/radar/conti_radar:install",
        "//modules/drivers/radar/racobit_radar:install",
        "//modules/drivers/radar/ultrasonic_radar:install",
        "//modules/drivers/video/tools/decode_video:install",
        ":pb_drivers_py"
    ],
)

install_files(
    name = "pb_drivers",
    dest = "drivers",
    files = [
        "//modules/common_msgs/sensor_msgs:ins_py_pb2",
        "//modules/common_msgs/sensor_msgs:conti_radar_py_pb2",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/sensor_msgs:sensor_image_py_pb2",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_py_pb2",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "drivers/include",
    data = [
        "//modules/drivers/camera/proto:config_cc_proto",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_py_pb2",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_cc_proto",
        "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
        "//modules/drivers/lidar/proto:robosense_config_cc_proto",
        "//modules/drivers/lidar/proto:robosense_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_cc_proto",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
        "//modules/drivers/radar/racobit_radar/proto:racobit_radar_conf_cc_proto",
        "//modules/drivers/radar/ultrasonic_radar/proto:ultrasonic_radar_conf_cc_proto",
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
        "//modules/drivers/video/proto:video_h265cfg_cc_proto",

    ],
)

install_files(
    name = "pb_drivers_py",
    dest = "drivers/python/modules/drivers",
    files = [
        "//modules/drivers/camera/proto:config_py_pb2",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_py_pb2",
        "//modules/drivers/gnss/proto:config_py_pb2",
        "//modules/drivers/gnss/proto:gnss_status_py_pb2",
        "//modules/drivers/lidar/proto:config_py_pb2",
        "//modules/drivers/lidar/proto:hesai_config_py_pb2",
        "//modules/drivers/lidar/proto:hesai_py_pb2",
        "//modules/drivers/lidar/proto:lidar_parameter_py_pb2",
        "//modules/drivers/lidar/proto:robosense_config_py_pb2",
        "//modules/drivers/lidar/proto:robosense_py_pb2",
        "//modules/drivers/lidar/proto:velodyne_py_pb2",
        "//modules/drivers/lidar/proto:velodyne_config_py_pb2",
        #"//modules/drivers/lidar/robosense/proto:lidars_filter_config_py_pb2",
        #"//modules/drivers/lidar/robosense/proto:sensor_suteng_conf_py_pb2",
        #"//modules/drivers/lidar/robosense/proto:sensor_suteng_py_pb2",
        "//modules/drivers/microphone/proto:microphone_config_py_pb2",
        "//modules/drivers/microphone/proto:audio_py_pb2",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_py_pb2",
        "//modules/drivers/radar/racobit_radar/proto:racobit_radar_conf_py_pb2",
        "//modules/drivers/radar/ultrasonic_radar/proto:ultrasonic_radar_conf_py_pb2",
        "//modules/drivers/smartereye/proto:config_py_pb2",
        "//modules/drivers/tools/image_decompress/proto:config_py_pb2",
        "//modules/drivers/video/proto:video_h265cfg_py_pb2"
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_drivers_src",
        ":install_drivers_hdrs"
    ],
)

install_src_files(
    name = "install_drivers_src",
    src_dir = ["."],
    dest = "drivers/src",
    filter = "*",
)

install_src_files(
    name = "install_drivers_hdrs",
    src_dir = ["."],
    dest = "drivers/include",
    filter = "*.h",
)