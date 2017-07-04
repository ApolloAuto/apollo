package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":log",
        ":macro",
        "//third_party/ros:ros_common",
        "@glog//:glog",
    ],
)

cc_library(
    name = "macro",
    hdrs = [
        "macro.h",
    ],
)

cc_library(
    name = "log",
    hdrs = [
        "log.h",
    ],
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "apollo_app",
    srcs = [
        "apollo_app.cc",
    ],
    hdrs = [
        "apollo_app.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/hmi/utils:hmi_status_helper",
        "//third_party/ros:ros_common",
        "@glog//:glog",
    ],
)
