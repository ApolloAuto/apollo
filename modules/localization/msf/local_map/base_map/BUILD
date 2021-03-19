load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_map",
    srcs = [
        "base_map.cc",
        "base_map_config.cc",
        "base_map_matrix.cc",
        "base_map_node.cc",
        "base_map_node_index.cc",
        "base_map_pool.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/common/util",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:base_map_cache",
        "//modules/localization/msf/common/util:compression",
        "//modules/localization/msf/common/util:system_utility",
        "@boost",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "map_node_index_test",
    timeout = "short",
    srcs = ["map_node_index_test.cc"],
    deps = [
        ":base_map",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
