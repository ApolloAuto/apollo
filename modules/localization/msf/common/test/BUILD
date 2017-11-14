load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "localization_msf_common_test",
    size = "medium",
    timeout = "short",
    srcs = glob(["io/*.cc",
                "util/*.cc"]),
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@glog//:glog",
        "@gtest//:main"
    ],
)

cpplint()
