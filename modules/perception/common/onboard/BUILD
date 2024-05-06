load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_onboard",
    srcs = [
        "common_flags/common_flags.cc",
        "msg_buffer/msg_buffer.cc",
        "msg_serializer/msg_serializer.cc",
        "transform_wrapper/transform_wrapper.cc",
    ],
    hdrs = [
        "common_flags/common_flags.h",
        "msg_buffer/msg_buffer.h",
        "msg_serializer/msg_serializer.h",
        "transform_wrapper/transform_wrapper.h",
        "inner_component_messages/camera_detection_component_messages.h",
        "inner_component_messages/camera_inner_component_messages.h",
        "inner_component_messages/inner_component_messages.h",
        "inner_component_messages/lidar_inner_component_messages.h",
        "inner_component_messages/traffic_inner_component_messages.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_benchmark_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera_camera_frame",
        "//modules/perception/common/lidar:apollo_perception_common_lidar_common",
        "//modules/transform:apollo_transform",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package()

cpplint()
