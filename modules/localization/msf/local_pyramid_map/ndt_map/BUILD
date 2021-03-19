load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_map_config",
    srcs = ["ndt_map_config.cc"],
    hdrs = ["ndt_map_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "ndt_map_matrix",
    srcs = ["ndt_map_matrix.cc"],
    hdrs = ["ndt_map_matrix.h"],
    deps = [
        ":ndt_map_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_matrix",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "ndt_map_matrix_handler",
    srcs = ["ndt_map_matrix_handler.cc"],
    hdrs = ["ndt_map_matrix_handler.h"],
    deps = [
        ":ndt_map_config",
        ":ndt_map_matrix",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "ndt_map_node_config",
    srcs = ["ndt_map_node_config.cc"],
    hdrs = ["ndt_map_node_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node_config",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "ndt_map_node",
    srcs = ["ndt_map_node.cc"],
    hdrs = ["ndt_map_node.h"],
    deps = [
        ":ndt_map_config",
        ":ndt_map_matrix",
        ":ndt_map_matrix_handler",
        ":ndt_map_node_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "ndt_map_pool",
    srcs = ["ndt_map_pool.cc"],
    hdrs = ["ndt_map_pool.h"],
    deps = [
        ":ndt_map_matrix",
        ":ndt_map_matrix_handler",
        ":ndt_map_node",
        ":ndt_map_node_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_pool",
    ],
)

cc_library(
    name = "ndt_map",
    srcs = ["ndt_map.cc"],
    hdrs = ["ndt_map.h"],
    deps = [
        ":ndt_map_config",
        ":ndt_map_node",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "local_map/test_data/ndt_map/**",
    ]),
)

cc_test(
    name = "ndt_map_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_test.cc"],
    data = [":test_data"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/ndt_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:ndt_map_pool",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_config_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/local_pyramid_map/ndt_map:ndt_map_config",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cpplint()
