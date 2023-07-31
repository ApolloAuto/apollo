load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data_dest = "drivers-radar/addition_data/conti_radar",
#     library_dest = "drivers-radar/lib/conti_radar",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libconti_radar.so",
#     ],
#     deps = [
#         "pb_hdrs",
#         "pb_drivers_radar_py", 
#     ]
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-radar/include",
#     data = [
#         "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_radar_py",
#     dest = "drivers-radar/python/modules/drivers/radar",
#     files = [
#         "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_py_pb2",
#     ],
# )

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
    name = "conti_radar_message_manager",
    srcs = ["conti_radar_message_manager.cc"],
    hdrs = ["conti_radar_message_manager.h"],
    deps = [
        "//modules/common/util:util_tool",
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
    alwayslink = True,
    deps = [
        ":conti_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "@eigen",
    ],
)

cc_binary(
    name = "libconti_radar.so",
    linkshared = True,
    linkstatic = True,
    deps = [":conti_radar_canbus_lib"],
)

apollo_package()
cpplint()
