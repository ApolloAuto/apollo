load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_io",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//framework:cybertron",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
