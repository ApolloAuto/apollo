load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_camera_camera_frame",
    srcs = [
        "common/data_provider.cc",
        "common/image_data_operations.cc",
        "common/undistortion_handler.cc",
    ],
    hdrs = [
        "common/camera_frame.h",
        "common/data_provider.h",
        "common/image_data_operations.h",
        "common/undistortion_handler.h",
    ] + if_cuda([
        "common/image_data_operations_npp.h",
    ]) + if_rocm([
        "common/image_data_operations_rpp.h",
    ]),
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
    ] + if_cuda([
        "@npp",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rpp",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_common_camera",
    srcs = [
        "common/camera_ground_plane.cc",
        "common/camera_preprocess_general_util.cc",
        "common/cipv_camera.cc",
        "common/object_template_manager.cc",
        "common/pose.cc",
        "common/util.cc",
    ],
    hdrs = [
        "common/camera_ground_plane.h",
        "common/camera_preprocess_general_util.h",
        "common/cipv_camera.h",
        "common/global_config.h",
        "common/lane_object.h",
        "common/math_functions.h",
        "common/object_template_manager.h",
        "common/pose.h",
        "common/timer.h",
        "common/trafficlight_frame.h",
        "common/twod_threed_util.h",
        "common/util.h",
    ],
    deps = [
        ":apollo_perception_common_camera_camera_frame",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera/common/proto:object_template_meta_schema_cc_proto",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "@com_github_gflags_gflags//:gflags",
        "@npp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

apollo_package()

cpplint()
