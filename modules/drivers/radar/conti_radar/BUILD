load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "conti_radar_message_manager",
    srcs = ["conti_radar_message_manager.cc"],
    hdrs = ["conti_radar_message_manager.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
    ],
)

cc_library(
    name = "conti_radar_canbus_lib",
    srcs = ["conti_radar_canbus_component.cc"],
    hdrs = ["conti_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"conti_radar\\"'],
    deps = [
        ":conti_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:pose_proto",
        "@eigen",
    ],
)

cc_binary(
    name = "libconti_radar.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":conti_radar_canbus_lib"],
)

cpplint()
