load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_ndt_map",
    srcs = [
        "ndt_map.cc",
        "ndt_map_config.cc",
        "ndt_map_matrix.cc",
        "ndt_map_node.cc",
        "ndt_map_pool.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen",
    ],
)

filegroup(
    name = "localization_msf_ndt_map_test_data",
    srcs = glob([
        "local_map/test_data/ndt_map/**",
    ]),
)

cc_test(
    name = "localization_msf_ndt_map_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_test.cc"],
    data = [":localization_msf_ndt_map_test_data"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "localization_msf_ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_config_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cpplint()
