load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_type",
    hdrs = ["type.h"],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "lane_util",
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/lane_post_process/common:lane_type",
    ],
)

cpplint()
