load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ros_bridge",
    srcs = [
        "ros_bridge.cc",
    ],
    deps = [
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_manager",
        "//modules/control/common",
        "//modules/control/proto:control_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
