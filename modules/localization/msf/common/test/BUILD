load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "localization_msf_common_test_data",
    srcs = glob([
        "test_data/test_folder/*",
    ]),
)

cc_test(
    name = "localization_msf_common_test",
    size = "small",
    timeout = "short",
    srcs = glob([
        "io/*_test.cc",
        "util/*_test.cc",
    ]),
    data = [
        ":localization_msf_common_test_data",
    ],
    linkopts = [
        "-lz",
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@gtest//:main",
    ],
)

cpplint()
