load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "relative_obstacle",
    srcs = ["relative_obstacle.cc"],
    hdrs = ["relative_obstacle.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_library(
    name = "relative_obstacles_container",
    srcs = ["relative_obstacles_container.cc"],
    hdrs = ["relative_obstacles_container.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/proto:perception_proto",
        ":relative_obstacle",
    ],
)

cpplint()