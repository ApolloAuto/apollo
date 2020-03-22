load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_lossy_map",
    srcs = [
        "lossy_map_2d.cc",
        "lossy_map_config_2d.cc",
        "lossy_map_matrix_2d.cc",
        "lossy_map_pool_2d.cc",
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
        "local_map/test_data/**",
    ]),
)

cc_test(
    name = "localization_msf_lossy_map_config_2d_test",
    size = "medium",
    timeout = "short",
    srcs = ["lossy_map_config_2d_test.cc"],
    data = [
        ":localization_msf_local_map_test_data",
    ],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "//third_party:boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
