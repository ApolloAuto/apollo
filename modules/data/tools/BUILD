load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "event_collector_main",
    srcs = ["event_collector_main.cc"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "@ros//:ros_common",
    ],
)

cpplint()
