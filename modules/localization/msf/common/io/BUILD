load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_io",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "@eigen//:eigen",
        "@yaml_cpp//:yaml",
        "@pcl//:pcl",
    ],
)

cpplint()