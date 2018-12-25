load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_io",
    srcs = ["velodyne_utility.cc"],
    hdrs = glob(["*.h"]),
    deps = [
        "//cyber",
        "@eigen",
        "@pcl",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "localization_msf_velodyne_utility_test",
    size = "small",
    timeout = "short",
    srcs = ["velodyne_utility_test.cc"],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@gtest//:main",
    ],
)

cpplint()
