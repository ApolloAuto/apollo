load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine",
        "//modules/perception/lidar_detection/interface:base_lidar_detector",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
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
        "//modules/perception/common/base",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cuda_library(
    name = "feature_generator_cuda",
    srcs = ["feature_generator.cu"],
    hdrs = ["feature_generator.h"],
    deps = [
        ":util",
        "//modules/perception/common/base",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
    ],
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
        "//modules/perception/common/algorithm/io:io_util",
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
#        "//modules/perception/common/lidar/common:pcl_util",
#        "@com_google_googletest//:gtest_main",
#        "@opencv2//:core",
#        "@opencv2//:highgui",
#    ],
#)

apollo_package()
cpplint()
