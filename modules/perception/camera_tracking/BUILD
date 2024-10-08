load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_refinement_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

gpu_library(
    name = "similar_cuda",
    srcs = ["common/similar.cu"],
    hdrs = [
        "common/camera_tracking_frame.h",
        "common/similar.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_camera_tracking",
    srcs = [
        "base/obstacle_reference.cc",
        "base/target.cc",
        "common/kalman_filter.cc",
        "common/similar.cc",
        "feature_extract/external_feature_extractor.cc",
        "feature_extract/project_feature.cc",
        "feature_extract/tracking_feat_extractor.cc",
        "tracking/omt_obstacle_tracker.cc",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    hdrs = [
        "base/frame_list.h",
        "base/obstacle_reference.h",
        "base/target.h",
        "base/track_object.h",
        "common/kalman_filter.h",
        "feature_extract/external_feature_extractor.h",
        "feature_extract/project_feature.h",
        "feature_extract/tracking_feat_extractor.h",
        "interface/base_feature_extractor.h",
        "interface/base_obstacle_tracker.h",
        "tracking/omt_obstacle_tracker.h",
    ],
    deps = [
        ":similar_cuda",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/camera_tracking/feature_extract/proto:model_param_cc_proto",
        "//modules/perception/camera_tracking/proto:camera_tracking_component_cc_proto",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
        "//modules/perception/camera_tracking/proto:tracking_feature_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@boost",
        "@eigen",
    ],
)

apollo_component(
    name = "libcamera_tracking_component_camera.so",
    srcs = [
        "camera_tracking_component.cc",
    ],
    hdrs = [
        "camera_tracking_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        ":apollo_perception_camera_tracking",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/camera_tracking/proto:camera_tracking_component_cc_proto",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
        "//modules/perception/camera_tracking/proto:tracking_feature_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()

cpplint()
