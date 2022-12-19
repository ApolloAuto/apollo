load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/ultrasonic_radar",
    library_dest = "drivers/lib/racobit_radar",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libultrasonic_radar_canbus_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "ultrasonic_radar_message_manager",
    srcs = ["ultrasonic_radar_message_manager.cc"],
    hdrs = ["ultrasonic_radar_message_manager.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
    ],
)

#  cc_test(
#    name = "ultrasonic_radar_message_manager_test",
#    size = "small",
#     srcs = ["ultrasonic_radar_message_manager_test.cc"],
#     deps = [
#         "ultrasonic_radar_message_manager",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "ultrasonic_radar_canbus_lib",
    srcs = ["ultrasonic_radar_canbus.cc"],
    hdrs = ["ultrasonic_radar_canbus.h"],
    deps = [
        ":ultrasonic_radar_message_manager",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/ultrasonic_radar/proto:ultrasonic_radar_conf_cc_proto",
    ],
)

# cc_test(
#     name = "ultrasonic_radar_canbus_test",
#     size = "small",
#     srcs = ["ultrasonic_radar_canbus_test.cc"],
#     deps = [
#         ":ultrasonic_radar_canbus_lib",
#         ":ultrasonic_radar_message_manager",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "ultrasonic_radar_canbus_component_lib",
    srcs = ["ultrasonic_radar_canbus_component.cc"],
    hdrs = ["ultrasonic_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"ultrasonic_radar_canbus\\"'],
    alwayslink = True,
    deps = [
        ":ultrasonic_radar_canbus_lib",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
    ],
)

cc_binary(
    name = "libultrasonic_radar_canbus_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":ultrasonic_radar_canbus_component_lib"],
)

cpplint()
