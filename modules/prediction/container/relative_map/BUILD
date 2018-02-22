load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "relative_map_container",
    srcs = ["relative_map_container.cc"],
    hdrs = ["relative_map_container.h"],
    deps = [
        "//modules/common:log",
        "//modules/map/relative_map:relative_map_lib",
        "//modules/prediction/container",
    ],
)

cpplint()
