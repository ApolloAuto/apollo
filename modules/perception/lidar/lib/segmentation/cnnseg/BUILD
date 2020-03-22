load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

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
        "//modules/perception/base",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/caffe:caffe_net_lib",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_config_proto",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_param_proto",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:spp_engine_config_proto",
        "//modules/perception/lidar/lib/segmentation/cnnseg/spp_engine",
        "//modules/perception/lidar/lib/segmentation/ncut:ncut_segmentation",
        "//modules/perception/proto:perception_config_schema_proto",
        "@com_google_absl//absl/strings",
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
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_param_proto",
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
        "//modules/perception/base",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_param_proto",
        "@cuda",
        "@eigen",
    ],
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
)

cc_test(
    name = "cnn_segmentation_test",
    size = "small",
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
