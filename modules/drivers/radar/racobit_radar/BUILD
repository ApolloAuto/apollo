load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "racobit_radar_message_manager",
    srcs = [
        "racobit_radar_message_manager.cc",
    ],
    hdrs = [
        "racobit_radar_message_manager.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

# cc_test(
#     name = "racobit_radar_message_manager_test",
#     size = "small",
#     srcs = [
#         "racobit_radar_message_manager_test.cc",
#     ],
#     deps = [
#         "racobit_radar_message_manager",
#         "@gtest//:main",
#     ],
# )

cc_library(
    name = "racobit_radar_canbus_lib",
    srcs = [
        "racobit_radar_canbus.cc",
    ],
    hdrs = [
        "racobit_radar_canbus.h",
    ],
    deps = [
        ":racobit_radar_message_manager",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

# cc_test(
#     name = "racobit_radar_canbus_test",
#     size = "small",
#     srcs = ["racobit_radar_canbus_test.cc"],
#     deps = [
#         ":racobit_radar_canbus_lib",
#         ":racobit_radar_message_manager",
#         "//modules/drivers/proto:sensor_proto",
#         "@gtest//:main",
#     ],
# )

cc_binary(
    name = "racobit_radar",
    srcs = ["main.cc"],
    deps = [
        ":racobit_radar_canbus_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/common:canbus_common",
        "@ros//:ros_common",
    ],
)

cpplint()
