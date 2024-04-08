load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_ch",
    srcs = [
        "ch_controller.cc",
        "ch_message_manager.cc",
        "ch_vehicle_factory.cc",
        "protocol/brake_command_111.cc",
        "protocol/brake_status__511.cc",
        "protocol/ecu_status_1_515.cc",
        "protocol/ecu_status_2_516.cc",
        "protocol/ecu_status_3_517.cc",
        "protocol/ecu_status_4_518.cc",
        "protocol/gear_command_114.cc",
        "protocol/gear_status_514.cc",
        "protocol/steer_command_112.cc",
        "protocol/steer_status__512.cc",
        "protocol/throttle_command_110.cc",
        "protocol/throttle_status__510.cc",
        "protocol/turnsignal_command_113.cc",
        "protocol/turnsignal_status__513.cc",
        "protocol/vehicle_mode_command_116.cc",
        "protocol/vin_resp1_51b.cc",
        "protocol/vin_resp2_51c.cc",
        "protocol/vin_resp3_51d.cc",
        "protocol/wheelspeed_report_51e.cc",
    ],
    hdrs = [
        "ch_controller.h",
        "ch_message_manager.h",
        "ch_vehicle_factory.h",
        "protocol/brake_command_111.h",
        "protocol/brake_status__511.h",
        "protocol/ecu_status_1_515.h",
        "protocol/ecu_status_2_516.h",
        "protocol/ecu_status_3_517.h",
        "protocol/ecu_status_4_518.h",
        "protocol/gear_command_114.h",
        "protocol/gear_status_514.h",
        "protocol/steer_command_112.h",
        "protocol/steer_status__512.h",
        "protocol/throttle_command_110.h",
        "protocol/throttle_status__510.h",
        "protocol/turnsignal_command_113.h",
        "protocol/turnsignal_status__513.h",
        "protocol/vehicle_mode_command_116.h",
        "protocol/vin_resp1_51b.h",
        "protocol/vin_resp2_51c.h",
        "protocol/vin_resp3_51d.h",
        "protocol/wheelspeed_report_51e.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/ch/proto:ch_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libch_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_ch"],
)

apollo_cc_test(
    name = "ch_controller_test",
    size = "small",
    srcs = ["ch_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ch_message_manager_test",
    size = "small",
    srcs = ["ch_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ch_vehicle_factory_test",
    size = "small",
    srcs = ["ch_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_cc_test(
    name = "brake_status__511_test",
    size = "small",
    srcs = ["protocol/brake_status__511_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ecu_status_1_515_test",
    size = "small",
    srcs = ["protocol/ecu_status_1_515_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ecu_status_2_516_test",
    size = "small",
    srcs = ["protocol/ecu_status_2_516_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ecu_status_3_517_test",
    size = "small",
    srcs = ["protocol/ecu_status_3_517_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gear_command_114_test",
    size = "small",
    srcs = ["protocol/gear_command_114_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gear_status_514_test",
    size = "small",
    srcs = ["protocol/gear_status_514_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "steer_command_112_test",
    size = "small",
    srcs = ["protocol/steer_command_112_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "steer_status__512_test",
    size = "small",
    srcs = ["protocol/steer_status__512_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "throttle_command_110_test",
    size = "small",
    srcs = ["protocol/throttle_command_110_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "throttle_status__510_test",
    size = "small",
    srcs = ["protocol/throttle_status__510_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "turnsignal_command_113_test",
    size = "small",
    srcs = ["protocol/turnsignal_command_113_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "turnsignal_status__513_test",
    size = "small",
    srcs = ["protocol/turnsignal_status__513_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ch",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
