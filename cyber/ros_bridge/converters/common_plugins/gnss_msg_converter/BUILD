load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

apollo_plugin(
    name = "libheading_msg_fusion.so",
    srcs = ["heading_msg_fusion.cc"],
    hdrs = [
        "heading_msg_fusion.h",
        "quaternion_math.h",
    ],
    description = ":heading_msg_plugins.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "//modules/common_msgs/sensor_msgs:heading_proto",
        "@ros"
    ],
)

apollo_plugin(
    name = "libimu_msg_converter.so",
    srcs = ["imu_msg_converter.cc"],
    hdrs = [
        "imu_msg_converter.h",
        "quaternion_math.h",
    ],
    description = ":imu_plugins.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "//modules/common_msgs/localization_msgs:imu_proto",
        "//modules/common_msgs/sensor_msgs:imu_proto",
        "@ros"
    ],
)

apollo_plugin(
    name = "libnav_msg_converter.so",
    srcs = ["nav_msg_converter.cc"],
    hdrs = ["nav_msg_converter.h"],
    description = ":nav_plugins.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_proto",
        "//modules/common_msgs/sensor_msgs:ins_proto",
        "@ros"
    ],
)

apollo_plugin(
    name = "libodometry_msg_converter.so",
    srcs = ["odometry_msg_converter.cc"],
    hdrs = ["odometry_msg_converter.h"],
    description = ":odometry_msg_plugins.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "//modules/common_msgs/localization_msgs:gps_proto",
        "@ros"
    ],
)

apollo_plugin(
    name = "libodometry_parser.so",
    srcs = ["odometry_parser.cc"],
    hdrs = [
        "odometry_parser.h",
        "quaternion_math.h",
    ],
    description = ":odometry_plugins.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@ros",
        "@proj",
        "@eigen"
    ],
)

apollo_package()

cpplint()
