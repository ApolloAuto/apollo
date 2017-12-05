load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":log",
        ":macro",
        "@glog//:glog",
        "@ros//:ros_common",
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
        ":log",
        "//modules/common/status",
        "//modules/common/util:string_util",
        "@ros//:ros_common",
    ],
)

cpplint()
