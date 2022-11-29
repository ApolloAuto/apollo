load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib",
    targets = [
        ":libsensor_gflags.so",
        ":libsensor_canbus.so"
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libsensor_gflags.so",
    srcs = ["sensor_gflags.cc", "sensor_gflags.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_gflags",
    srcs = ["libsensor_gflags.so"],
    hdrs = ["sensor_gflags.h"],
    alwayslink = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_canbus_lib",
    srcs = ["libsensor_canbus.so"],
    alwayslink = True,
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cc_binary(
    name = "libsensor_canbus.so",
    srcs = ["sensor_canbus.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cpplint()
