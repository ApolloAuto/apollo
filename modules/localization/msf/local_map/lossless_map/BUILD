load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_lossless_map",
    srcs = [
        "lossless_map.cc",
        "lossless_map_config.cc",
        "lossless_map_matrix.cc",
        "lossless_map_node.cc",
        "lossless_map_pool.cc",
    ],
    hdrs = glob(["*.h"]),
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen",
    ],
)

filegroup(
    name = "localization_msf_local_map_test_data",
    srcs = glob([
        "modules/localization/msf/local_map/test_data/**",
    ]),
)

cc_test(
    name = "localization_msf_local_map_pool_test",
    size = "medium",
    timeout = "moderate",
    srcs = ["map_pool_test.cc"],
    copts = [
        "-Dprivate=public",
    ],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_msf_lossless_map_config_test",
    size = "medium",
    timeout = "moderate",
    srcs = ["lossless_map_config_test.cc"],
    data = [":localization_msf_local_map_test_data"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
