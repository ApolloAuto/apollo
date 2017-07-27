load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_manager",
    srcs = ["container_manager.cc"],
    hdrs = ["container_manager.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common:log",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "@glog//:glog",
    ],
)

cc_library(
    name = "container",
    hdrs = ["container.h"],
    deps = [
        "@com_github_google_protobuf//:protobuf",
    ],
)

cpplint()
