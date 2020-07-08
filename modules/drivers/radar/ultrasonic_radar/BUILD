load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ultrasonic_radar_message_manager",
    srcs = ["ultrasonic_radar_message_manager.cc"],
    hdrs = ["ultrasonic_radar_message_manager.h"],
    deps = [
        "//modules/common/proto:header_cc_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/proto:ultrasonic_radar_cc_proto",
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
    deps = [
        ":ultrasonic_radar_canbus_lib",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/proto:ultrasonic_radar_cc_proto",
    ],
)

cc_binary(
    name = "libultrasonic_radar_canbus_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":ultrasonic_radar_canbus_component_lib"],
)

cpplint()
