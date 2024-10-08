load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"trafficlight\\"']

filegroup(
    name = "traffic_light_region_proposal_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_traffic_light_region_proposal",
    srcs = [
        "preprocessor/multi_camera_projection.cc",
        "preprocessor/tl_preprocessor.cc",
    ],
    hdrs = [
        "interface/base_tl_preprocessor.h",
        "preprocessor/multi_camera_projection.h",
        "preprocessor/tl_preprocessor.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:util_tool",
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
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_region_proposal/proto:trafficlights_proposal_component_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_component(
    name = "libtraffic_component_proposal.so",
    srcs = [
        "traffic_light_region_proposal_component.cc",
    ],
    hdrs = [
        "traffic_light_region_proposal_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lpthread"],
    deps = [
        "//cyber",
        ":apollo_perception_traffic_light_region_proposal",
        "//modules/common/math",
        "//modules/common/util:util_tool",
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
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_region_proposal/proto:trafficlights_proposal_component_cc_proto",
        "@eigen",
    ],
)

apollo_package()

cpplint()
