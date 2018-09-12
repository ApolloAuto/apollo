load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "conti_radar_message_manager",
    srcs = [
        "conti_radar_message_manager.cc",
    ],
    hdrs = [
        "conti_radar_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
        "//modules/common/util:message_util"
    ],
)

cc_library(
    name = "conti_radar_canbus_lib",
    srcs = [
        "conti_radar_canbus_component.cc",
    ],
    hdrs = [
        "conti_radar_canbus_component.h",
    ],
    deps = [
        ":conti_radar_message_manager",
        "//modules/common",
        # "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
    ],
    copts = ['-DMODULE_NAME=\\"conti_radar\\"']
)

cc_binary(
    name = "libconti_radar.so",
    deps = [":conti_radar_canbus_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cpplint()
