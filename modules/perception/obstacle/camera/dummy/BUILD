load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy",
    srcs = ["dummy_algorithms.cc"],
    hdrs = ["dummy_algorithms.h"],
    deps = [
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/camera/common:projector",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/obstacle/common",
    ],
)

cpplint()
