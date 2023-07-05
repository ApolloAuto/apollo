load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/racobit_radar",
    library_dest = "drivers/lib/racobit_radar",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libracobit_radar_canbus_component.so",
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
    name = "racobit_radar_message_manager",
    srcs = ["racobit_radar_message_manager.cc"],
    hdrs = ["racobit_radar_message_manager.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/common_msgs/sensor_msgs:racobit_radar_cc_proto",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

# cc_test(
#     name = "racobit_radar_message_manager_test",
#     size = "small",
#     srcs = ["racobit_radar_message_manager_test.cc"],
#     deps = [
#         "racobit_radar_message_manager",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "racobit_radar_canbus_lib",
    srcs = ["racobit_radar_canbus_component.cc"],
    hdrs = ["racobit_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"racobit_radar_canbus\\"'],
    alwayslink = True,
    deps = [
        ":racobit_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

cc_binary(
    name = "libracobit_radar_canbus_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":racobit_radar_canbus_lib"],
)

cpplint()
