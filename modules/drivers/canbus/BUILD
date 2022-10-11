load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib",
    targets = [
        ":libsensor_gflags.so",
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
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_canbus_lib",
    hdrs = ["sensor_canbus.h"],
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cpplint()
