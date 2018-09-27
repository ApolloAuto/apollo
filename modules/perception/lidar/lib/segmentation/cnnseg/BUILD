load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cnn_segmentation",
    srcs = [
        "cnn_segmentation.cc",
    ],
    hdrs = [
        "cnn_segmentation.h",
    ],
    deps = [
        ":feature_generator",
        "//framework:cybertron",
        "//modules/perception/base",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_config_proto",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:spp_engine_config_proto",
        "//modules/perception/lidar/lib/segmentation/cnnseg/spp_engine",
        "//modules/perception/proto:perception_config_schema_proto",
    ],
)

cc_library(
    name = "disjoint_set",
    hdrs = [
        "disjoint_set.h",
    ],
)

cc_library(
    name = "feature_generator",
    srcs = [
        "feature_generator.cc",
    ],
    hdrs = [
        "feature_generator.h",
    ],
    deps = [
        ":util",
        "//modules/perception/base",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_param_proto",
        "@eigen",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
)

cpplint()
