load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

SERIAL_COPTS = ["-DMODULE_NAME=\\\"serial\\\""]

apollo_component(
    name = "libserial_component.so",
    srcs = ["serial_component.cc"],
    hdrs = ["serial_component.h",],
    copts = SERIAL_COPTS,
    deps = [
        ":apollo_serial_lib",
        "//cyber",
    ],
)

apollo_cc_library(
    name = "apollo_serial_lib",
    hdrs = [
        "vehicle_factory.h",
        "base_control.h",
    ],
    copts = SERIAL_COPTS,
    deps = [
        ":ros_control",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
    ],
)

apollo_cc_library(
    name = "ros_control",
    hdrs = [
        "base_control.h",
        "vehicle/ros/ros_control.h",
        "vehicle/ros/ros_parser.h",
        "vehicle/ros/protocol/twist_fb.h",
        "vehicle/ros/protocol/twist_cmd.h",
        "vehicle/ros/protocol/misc_fb.h",
        "common/util.h",
        "common/serial_stream.h",
        "common/ring_buffer.h",
    ],
    srcs = [
        "vehicle/ros/ros_control.cc",
        "vehicle/ros/ros_parser.cc",
        "vehicle/ros/protocol/twist_fb.cc",
        "vehicle/ros/protocol/twist_cmd.cc",
        "vehicle/ros/protocol/misc_fb.cc",
        "common/serial_stream.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus:byte",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/serial/proto:serial_conf_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
