load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_common",
    srcs = [
        "geometry_util.cc",
    ],
    hdrs = [
        "convex_hullxy.h",
        "disjoint_set.h",
        "geometry_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/pcl_util:pcl_util",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "perception_obstacle_common_test",
    size = "small",
    srcs = [
        "geometry_util_test.cc",
    ],
    deps = [
        ":perception_obstacle_common",
        "@gtest//:main"
    ],
)

cpplint()

