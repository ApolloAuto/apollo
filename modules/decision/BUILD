load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_decision",
    srcs = ["decision.cc"],
    hdrs = ["decision.h"],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/time",
        "//modules/decision/common:decision_common",
        "//modules/decision/proto:decision_proto",
        "//third_party/ros:ros_common",
        "@glog//:glog",
    ],
)

cc_binary(
    name = "decision",
    srcs = ["main.cc"],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/decision:lib_decision",
        "//modules/decision/common:decision_common",
        "//modules/decision/proto:decision_proto",
        "//third_party/ros:ros_common",
    ],
)

cpplint()
