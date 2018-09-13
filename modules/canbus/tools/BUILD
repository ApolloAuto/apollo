load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "canbus_tester",
    srcs = ["canbus_tester.cc"],
    deps = [
        "//external:gflags",
        "//modules/canbus:canbus_lib",
        "//modules/canbus/common:canbus_common",
        "//modules/common",
        "//framework:cybertron",
    ],
)

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/control/proto:control_proto",
        "//framework:cybertron",
    ],
)

cpplint()
