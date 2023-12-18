load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
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
    ]),
)

apollo_cc_library(
    name = "apollo_perception_lane_detection",
    srcs = [
        "app/debug_info.cc",
        "app/lane_camera_perception.cc",
        "common/common_functions.cc",
        "lib/detector/darkSCNN/darkSCNN_lane_detector.cc",
        "lib/detector/denseline/denseline_lane_detector.cc",
        "lib/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.cc",
        "lib/postprocessor/denseline/denseline_lane_postprocessor.cc",
    ],
    hdrs = [
        "app/debug_info.h",
        "app/lane_camera_perception.h",
        "common/common_functions.h",
        "interface/base_camera_perception.h",
        "interface/base_lane_detector.h",
        "interface/base_lane_postprocessor.h",
        "lib/detector/darkSCNN/darkSCNN_lane_detector.h",
        "lib/detector/denseline/denseline_lane_detector.h",
        "lib/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h",
        "lib/postprocessor/denseline/denseline_lane_postprocessor.h",
    ],
    deps = [
        "//cyber",
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
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "//modules/perception/lane_detection/proto:darkSCNN_cc_proto",
        "//modules/perception/lane_detection/proto:darkSCNN_postprocessor_cc_proto",
        "//modules/perception/lane_detection/proto:denseline_cc_proto",
        "//modules/perception/lane_detection/proto:denseline_postprocessor_cc_proto",
        "//modules/perception/lane_detection/proto:lane_perception_component_cc_proto",
        "//modules/perception/lane_detection/proto:perception_cc_proto",
        "//modules/perception/tools/offline:perception_transform_server",
        "//modules/perception/tools/offline:perception_visualizer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
        "@opencv//:highgui",
    ],
)

apollo_component(
    name = "liblane_detection.so",
    srcs = [
        "lane_detection_component.cc",
    ],
    hdrs = [
        "lane_detection_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        ":apollo_perception_lane_detection",
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
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "//modules/perception/lane_detection/proto:lane_perception_component_cc_proto",
        "//modules/perception/tools/offline:perception_transform_server",
        "//modules/perception/tools/offline:perception_visualizer",
        "@eigen",
        "@opencv//:highgui",
    ],
)

apollo_package()

cpplint()
