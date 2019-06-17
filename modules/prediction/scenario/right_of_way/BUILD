load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "right_of_way",
    srcs = ["right_of_way.cc"],
    hdrs = ["right_of_way.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
    ],
)

cpplint()
