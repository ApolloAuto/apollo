load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_gflags",
    srcs = [
        "camera_gflags.cc",
    ],
    hdrs = [
        "camera_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "camera_common",
    deps = [
        ":camera_gflags",
    ],
)
cpplint()
