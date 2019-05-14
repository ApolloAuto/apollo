load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fem_pose_deviation_smoother",
    srcs = [
        "fem_pose_deviation_smoother.cc",
    ],
    hdrs = [
        "fem_pose_deviation_smoother.h",
    ],
    deps = [
        "//cyber/common:log",
        "@osqp",
    ],
)

cpplint()
