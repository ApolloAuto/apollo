package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_manager",
    srcs = ["container_manager.cc"],
    hdrs = ["container_manager.h"],
    deps = [
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "//modules/common:macro",
        "@glog//:glog"
    ]
)

cc_library(
    name = "container",
    hdrs = ["container.h"],
    deps = [
        "@com_github_google_protobuf//:protobuf"
    ]
)