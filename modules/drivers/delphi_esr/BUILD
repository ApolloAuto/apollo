load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "delphi_esr_message_manager",
    srcs = [
    ],
    hdrs = [
        "delphi_esr_message_manager.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/delphi_esr/protocol:drivers_delphi_esr_protocol",
    ],
)

cc_test(
    name = "delphi_esr_message_manager_test",
    size = "small",
    srcs = [
        "delphi_esr_message_manager_test.cc",
    ],
    deps = [
        "delphi_esr_message_manager",
        "@gtest//:main",
    ],
)

cc_library(
    name = "delphi_esr_canbus_lib",
    srcs = [
    ],
    hdrs = [
        "delphi_esr_canbus.h",
    ],
    deps = [
        ":delphi_esr_message_manager",
        "//modules/drivers/canbus:sensor_canbus_lib",
        "//modules/drivers/delphi_esr/protocol:drivers_delphi_esr_protocol",
    ],
)

cc_test(
    name = "delphi_esr_canbus_test",
    size = "small",
    srcs = ["delphi_esr_canbus_test.cc"],
    deps = [
        ":delphi_esr_canbus_lib",
        ":delphi_esr_message_manager",
        "//modules/drivers/proto:sensor_proto",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "delphi_esr",
    srcs = ["main.cc"],
    deps = [
        ":delphi_esr_canbus_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor",
        "//modules/drivers/canbus/common:canbus_common",
        "@ros//:ros_common",
    ],
)

cpplint()
