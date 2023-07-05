load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cnn_segmentation",
    srcs = ["cnn_segmentation.cc"],
    hdrs = ["cnn_segmentation.h"],
    deps = [
        ":disjoint_set",
        ":feature_generator",
        ":util",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/base",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
    ] + if_cuda([
        "//modules/perception/inference/tensorrt:rt_net",
    ]) + if_rocm([
        "//modules/perception/inference/migraphx:mi_net",
    ]) + [
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/pipeline/proto/stage:cnnseg_config_cc_proto",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:cnnseg_param_cc_proto",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:spp_engine_config_cc_proto",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/spp_engine",
        "//modules/perception/lidar/lib/detector/ncut_segmentation:ncut_segmentation",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/pipeline:stage",
        "@com_google_absl//:absl",
    ],
    alwayslink = True,
)

cc_library(
    name = "disjoint_set",
    hdrs = ["disjoint_set.h"],
)

cc_library(
    name = "feature_generator",
    srcs = [
        "feature_generator.cc",
        ":feature_generator_cuda",
    ],
    hdrs = ["feature_generator.h"],
    deps = [
        ":util",
        "//modules/perception/base",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:cnnseg_param_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

gpu_library(
    name = "feature_generator_cuda",
    srcs = ["feature_generator.cu"],
    hdrs = ["feature_generator.h"],
    deps = [
        ":util",
        "//modules/perception/base",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:cnnseg_param_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
)

cc_test(
    name = "cnn_segmentation_test",
    size = "large",
    srcs = ["cnn_segmentation_test.cc"],
    deps = [
        ":cnn_segmentation",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

#cc_test(
#    name = "feature_generator_test",
#    size = "small",
#    srcs = ["feature_generator_test.cc"],
#    deps = [
#        ":feature_generator",
#        "//modules/perception/common:perception_gflags",
#        "//modules/perception/lidar/common:pcl_util",
#        "@com_google_googletest//:gtest_main",
#        "@opencv2//:core",
#        "@opencv2//:highgui",
#    ],
#)

cpplint()
