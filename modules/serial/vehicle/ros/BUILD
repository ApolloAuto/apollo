load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"serial\\\""]

apollo_cc_library(
    name = "ros_control",
    hdrs = [
        "ros_control.h",
        "base_control.h",
    ],
    srcs = [
        "ros_control.cc",
    ],
    deps = [
        ":ros_parser",
        "//cyber",
        "//modules/serial/common:ring_buffer",
        "//modules/serial/common:serial_stream",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
    ],
)

apollo_cc_library(
    name = "ros_parser",
    hdrs = [
        "ros_parser.h",
    ],
    srcs = [
        "ros_parser.cc",
    ],
    deps = [
        ":misc_fb",
        ":twist_cmd",
        ":twist_fb",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

apollo_cc_library(
    name = "misc_fb",
    hdrs = [
        "protocol/misc_fb.h",
    ],
    srcs = [
        "protocol/misc_fb.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus:byte",
    ],
)

apollo_cc_library(
    name = "twist_cmd",
    hdrs = [
        "protocol/twist_cmd.h",
    ],
    srcs = [
        "protocol/twist_cmd.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus:byte",
    ],
)

apollo_cc_library(
    name = "twist_fb",
    hdrs = [
        "protocol/twist_fb.h",
    ],
    srcs = [
        "protocol/twist_fb.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus:byte",
    ],
)

apollo_package()
cpplint()
