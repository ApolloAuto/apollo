load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus",
    srcs = [
        "common/canbus_gflags.cc",
        "vehicle/abstract_vehicle_factory.cc",
    ],
    hdrs = [
        "common/canbus_gflags.h",
        "vehicle/abstract_vehicle_factory.h",
        "vehicle/vehicle_controller.h",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "canbus_test",
    size = "small",
    srcs = ["canbus_test.cc"],
    deps = [
        ":DO_NOT_IMPORT_canbus_component",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_component(
    name = "libcanbus_component.so",
    srcs = ["canbus_component.cc"],
    hdrs = ["canbus_component.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":apollo_canbus",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "@com_github_gflags_gflags//:gflags",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

#TODO(storypku):
# split test_data for each vehicle
filegroup(
    name = "test_data",
    srcs = glob(["testdata/**"]),
)

apollo_package()
cpplint()
