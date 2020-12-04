load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_map_config",
    srcs = ["base_map_config.cc"],
    hdrs = [
        "base_map_config.h",
        "base_map_fwd.h",
    ],
    deps = [
        "//cyber/common",
        "//modules/common/util",
        "//modules/localization/msf/common/util",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "base_map_matrix",
    srcs = ["base_map_matrix.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_matrix.h",
    ],
    deps = [
        "//cyber/common",
        "@com_github_google_glog//:glog",
        "@eigen",
        "@opencv//:imgcodecs",
        "@zlib",
    ],
)

cc_library(
    name = "base_map_node_index",
    srcs = ["base_map_node_index.cc"],
    hdrs = [
        "base_map_config.h",
        "base_map_fwd.h",
        "base_map_node_index.h",
    ],
    deps = [
        ":base_map_config",
        "//cyber/common",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "base_map_node_config",
    srcs = ["base_map_node_config.cc"],
    hdrs = ["base_map_node_config.h"],
    deps = [
        ":base_map_node_index",
        "//cyber/common",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "base_map_node",
    srcs = ["base_map_node.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_matrix_handler.h",
        "base_map_node.h",
    ],
    deps = [
        ":base_map_config",
        ":base_map_matrix",
        ":base_map_node_config",
        ":base_map_node_index",
        "//cyber/common",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:compression",
        "//modules/localization/msf/common/util:file_utility",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "base_map_pool",
    srcs = ["base_map_pool.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_pool.h",
    ],
    deps = [
        ":base_map_config",
        ":base_map_node",
        ":base_map_node_index",
        "//cyber/common",
        "//modules/localization/msf/common/util",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "base_map",
    srcs = ["base_map.cc"],
    hdrs = [
        "base_map.h",
        "base_map_fwd.h",
    ],
    deps = [
        ":base_map_config",
        ":base_map_node",
        ":base_map_node_index",
        ":base_map_pool",
        "//cyber/common",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:base_map_cache",
        "//modules/localization/msf/common/util:file_utility",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cpplint()
