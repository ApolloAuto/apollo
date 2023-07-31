load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "traffic_light_region_proposal_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libtraffic_component_proposal.so",
    linkshared = True,
    linkstatic = True,
    deps = [":perception_component_traffic_proposal"],
)

cc_library(
    name = "perception_component_traffic_proposal",
    srcs = [
        "traffic_light_region_proposal_component.cc",
    ],
    hdrs = [
        "traffic_light_region_proposal_component.h",
    ],
    linkopts = ["-lpthread"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
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
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/traffic_light_region_proposal/interface:base_tl_preprocessor",
        "//modules/perception/traffic_light_region_proposal/preprocessor:tl_preprocessor",
        "//modules/perception/common/hdmap:hdmap_input",
        "//modules/perception/common/onboard/common_flags",
        "//modules/perception/common/onboard/msg_serializer",
        "//modules/perception/traffic_light_region_proposal/proto:trafficlights_proposal_component_cc_proto",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/perception/common/onboard/inner_component_messages:traffic_inner_component_messages",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
