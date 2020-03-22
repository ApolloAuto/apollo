load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spp_engine",
    srcs = ["spp_engine.cc"],
    hdrs = ["spp_engine.h"],
    deps = [
        ":spp_cluster",
        ":spp_cluster_list",
        ":spp_label_image",
        ":spp_seg_cc_2d",
        ":spp_struct",
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "spp_cluster",
    hdrs = ["spp_cluster.h"],
    deps = [
        "//modules/perception/base",
    ],
)

cc_library(
    name = "spp_cluster_list",
    srcs = ["spp_cluster_list.cc"],
    hdrs = ["spp_cluster_list.h"],
    deps = [
        ":spp_cluster",
        ":spp_label_image",
        ":spp_pool_types",
    ],
)

cc_test(
    name = "spp_cluster_list_test",
    size = "small",
    srcs = ["spp_cluster_list_test.cc"],
    deps = [
        ":spp_cluster_list",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "spp_label_image",
    srcs = ["spp_label_image.cc"],
    hdrs = ["spp_label_image.h"],
    deps = [
        ":spp_cluster",
        ":spp_pool_types",
        "//modules/perception/base",
        "//modules/perception/common/i_lib",
        "//modules/perception/lidar/common",
    ],
)

cc_library(
    name = "spp_pool_types",
    srcs = ["spp_pool_types.cc"],
    hdrs = ["spp_pool_types.h"],
    deps = [
        ":spp_cluster",
        "//modules/perception/base",
        "//modules/perception/lidar/common",
    ],
)

cc_library(
    name = "spp_seg_cc_2d",
    srcs = ["spp_seg_cc_2d.cc"],
    hdrs = ["spp_seg_cc_2d.h"],
    deps = [
        ":spp_label_image",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/common/i_lib",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
    ],
)

cc_library(
    name = "spp_struct",
    srcs = ["spp_struct.cc"],
    hdrs = ["spp_struct.h"],
    deps = [
        "//modules/perception/base",
    ],
)

cpplint()
