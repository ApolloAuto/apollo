load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lane_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "liblane_detection.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lane_detecton_component"],
)

cc_library(
    name = "lane_detecton_component",
    srcs = [
        "lane_detection_component.cc",
    ],
    hdrs = [
        "lane_detection_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_camera_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_lane_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:cipv_camera",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/onboard/common_flags",
        "//modules/perception/common/onboard/inner_component_messages",
        "//modules/perception/common/onboard/msg_serializer",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "//modules/perception/lane_detection/app:lane_camera_perception",
        "//modules/perception/lane_detection/interface",
        "//modules/perception/lane_detection/proto:lane_perception_component_cc_proto",
        "//modules/perception/tools/offline:transform_server",
        "//modules/perception/tools/offline:visualizer",
        "@eigen",
        "@opencv//:highgui",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
