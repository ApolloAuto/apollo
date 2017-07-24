load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_common",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "@eigen//:eigen"
    ],
)

cpplint()
