load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lossy_map",
    srcs = [
        "lossy_map_2d.cc",
        "lossy_map_config_2d.cc",
        "lossy_map_matrix_2d.cc",
        "lossy_map_pool_2d.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:compression",
        "//modules/localization/msf/local_map/base_map",
        "@eigen",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "local_map/test_data/**",
    ]),
)

cc_test(
    name = "lossy_map_config_2d_test",
    size = "medium",
    timeout = "short",
    srcs = ["lossy_map_config_2d_test.cc"],
    data = [
        ":test_data",
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

cpplint()
