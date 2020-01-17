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

cpplint()
