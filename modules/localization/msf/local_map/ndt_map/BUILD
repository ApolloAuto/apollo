load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_map",
    srcs = [
        "ndt_map.cc",
        "ndt_map_config.cc",
        "ndt_map_matrix.cc",
        "ndt_map_node.cc",
        "ndt_map_pool.cc",
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
        ":ndt_map",
        "//cyber",
        "//modules/localization/msf/common/io:common_io",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_config_test.cc"],
    deps = [
        ":ndt_map",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
