load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lossless_map",
    srcs = [
        "lossless_map.cc",
        "lossless_map_config.cc",
        "lossless_map_matrix.cc",
        "lossless_map_node.cc",
        "lossless_map_pool.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:base_map_cache",
        "//modules/localization/msf/common/util:compression",
        "//modules/localization/msf/local_map/base_map",
        "@eigen",
    ],
)

filegroup(
    name = "local_map_test_data",
    srcs = glob([
        "modules/localization/msf/local_map/test_data/**",
    ]),
)

cc_test(
    name = "map_pool_test",
    size = "small",
    srcs = ["map_pool_test.cc"],
    copts = [
        "-fno-access-control",
    ],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_map/base_map",
        "//modules/localization/msf/local_map/lossless_map",
        "//modules/localization/msf/local_map/lossy_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lossless_map_config_test",
    size = "small",
    srcs = ["lossless_map_config_test.cc"],
    data = [":local_map_test_data"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_map/base_map",
        "//modules/localization/msf/local_map/lossless_map",
        "//modules/localization/msf/local_map/lossy_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
