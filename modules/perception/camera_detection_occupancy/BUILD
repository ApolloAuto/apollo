load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_detection_occupancy",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_camera_detection_occupancy",
    srcs = [
        "detector/bevformer/bevformer_obstacle_detector.cc",
        "detector/bevformer/preprocess.cc",
        "tracker/camera_tracker/camera_tracker.cc",
        "tracker/common/camera_track_manager.cc",
        "tracker/common/camera_track.cc",
        "tracker/filter/adaptive_kalman_filter.cc",
        "tracker/matcher/hm_matcher.cc",
        "interface/base_obstacle_detector.cc",
        "interface/base_matcher.cc",
    ],
    hdrs = [
        "camera_frame.h",
        "detector/bevformer/bevformer_obstacle_detector.h",
        "detector/bevformer/preprocess.h",
        "tracker/camera_tracker/camera_tracker.h",
        "tracker/common/camera_track_manager.h",
        "tracker/common/camera_track.h",
        "tracker/filter/adaptive_kalman_filter.h",
        "tracker/matcher/hm_matcher.h",
        "interface/base_obstacle_detector.h",
        "interface/base_matcher.h",
        "interface/base_filter.h",
        "interface/base_tracker.h",
        "utils/utils.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_occupancy/detector/bevformer/proto:model_param_cc_proto",
        "//modules/perception/camera_detection_occupancy/proto:matcher_config_proto",
        "//modules/perception/camera_detection_occupancy/proto:tracker_config_proto",
        "//modules/perception/camera_detection_occupancy/proto:camera_detection_occupancy_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/common/proto:model_process_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@opencv//:imgproc",
    ],
)

apollo_cc_library(
    name = "camera_detection_occupancy_component",
    srcs = [
        "camera_detection_occupancy_component.cc",
    ],
    hdrs = [
        "camera_detection_occupancy_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        ":apollo_perception_camera_detection_occupancy",
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_occupancy/proto:matcher_config_proto",
        "//modules/perception/camera_detection_occupancy/proto:tracker_config_proto",
        "//modules/perception/camera_detection_occupancy/proto:camera_detection_occupancy_cc_proto",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

apollo_package()

cpplint()
