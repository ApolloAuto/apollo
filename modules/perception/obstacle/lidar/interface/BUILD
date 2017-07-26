load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_interface",
    srcs = [],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@eigen//:eigen",
    ],
)

cpplint()
