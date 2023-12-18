load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

apollo_cc_library(
    name = "msgexporterlib",
    srcs = ["msg_exporter.cc"],
    hdrs = ["msg_exporter.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "@com_google_absl//:absl",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
        "@local_config_pcl//:pcl",
    ],
)

apollo_cc_binary(
    name = "msg_exporter",
    srcs = ["msg_exporter_main.cc"],
    deps = [
        ":msgexporterlib",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "@com_google_absl//:absl",
    ],
)

apollo_package()
cpplint()
