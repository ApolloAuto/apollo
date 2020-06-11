load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_util",
    srcs = ["radar_util.cc"],
    hdrs = ["radar_util.h"],
    deps = [
        "//modules/perception/base",
        "@eigen",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)

cpplint()
