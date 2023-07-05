load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pyramid_map_config",
    srcs = ["pyramid_map_config.cc"],
    hdrs = ["pyramid_map_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "pyramid_map_matrix",
    srcs = ["pyramid_map_matrix.cc"],
    hdrs = [
        "aligned_matrix.h",
        "pyramid_map_matrix.h",
    ],
    deps = [
        ":pyramid_map_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_matrix",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "pyramid_map_matrix_handler",
    srcs = ["pyramid_map_matrix_handler.cc"],
    hdrs = ["pyramid_map_matrix_handler.h"],
    deps = [
        ":pyramid_map_config",
        ":pyramid_map_matrix",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "pyramid_map_node_config",
    srcs = ["pyramid_map_node_config.cc"],
    hdrs = ["pyramid_map_node_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node_config",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "pyramid_map_node",
    srcs = ["pyramid_map_node.cc"],
    hdrs = ["pyramid_map_node.h"],
    deps = [
        ":pyramid_map_config",
        ":pyramid_map_matrix",
        ":pyramid_map_matrix_handler",
        ":pyramid_map_node_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_node",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "pyramid_map_pool",
    srcs = ["pyramid_map_pool.cc"],
    hdrs = ["pyramid_map_pool.h"],
    deps = [
        ":pyramid_map_matrix",
        ":pyramid_map_matrix_handler",
        ":pyramid_map_node",
        ":pyramid_map_node_config",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:base_map_cache",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:base_map_pool",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "pyramid_map",
    srcs = ["pyramid_map.cc"],
    hdrs = ["pyramid_map.h"],
    deps = [
        ":pyramid_map_config",
        ":pyramid_map_node",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:base_map_cache",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_test(
    name = "pyramid_map_config_test",
    size = "medium",
    timeout = "short",
    srcs = ["pyramid_map_config_test.cc"],
    deps = [
        ":pyramid_map_config",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_map_matrix_handler_test",
    size = "medium",
    timeout = "short",
    srcs = ["pyramid_map_matrix_handler_test.cc"],
    deps = [
        ":pyramid_map_matrix_handler",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_map_matrix_test",
    size = "medium",
    timeout = "short",
    srcs = ["pyramid_map_matrix_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":pyramid_map_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_map_node_test",
    size = "medium",
    timeout = "short",
    srcs = ["pyramid_map_node_test.cc"],
    deps = [
        ":pyramid_map_node",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_map_pool_test",
    size = "large",
    timeout = "short",
    srcs = ["pyramid_map_pool_test.cc"],
    deps = [
        ":pyramid_map",
        ":pyramid_map_pool",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_map_test",
    size = "large",
    timeout = "short",
    srcs = ["pyramid_map_test.cc"],
    deps = [
        ":pyramid_map",
        ":pyramid_map_pool",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
