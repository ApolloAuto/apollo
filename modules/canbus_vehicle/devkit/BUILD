load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_devkit",
    srcs = [
        "devkit_controller.cc",
        "devkit_message_manager.cc",
        "devkit_vehicle_factory.cc",
        "protocol/bms_report_512.cc",
        "protocol/brake_command_101.cc",
        "protocol/brake_report_501.cc",
        "protocol/gear_command_103.cc",
        "protocol/gear_report_503.cc",
        "protocol/park_command_104.cc",
        "protocol/park_report_504.cc",
        "protocol/steering_command_102.cc",
        "protocol/steering_report_502.cc",
        "protocol/throttle_command_100.cc",
        "protocol/throttle_report_500.cc",
        "protocol/ultr_sensor_1_507.cc",
        "protocol/ultr_sensor_2_508.cc",
        "protocol/ultr_sensor_3_509.cc",
        "protocol/ultr_sensor_4_510.cc",
        "protocol/ultr_sensor_5_511.cc",
        "protocol/vcu_report_505.cc",
        "protocol/vehicle_mode_command_105.cc",
        "protocol/vin_resp1_514.cc",
        "protocol/vin_resp2_515.cc",
        "protocol/vin_resp3_516.cc",
        "protocol/wheelspeed_report_506.cc",
    ],
    hdrs = [
        "devkit_controller.h",
        "devkit_message_manager.h",
        "devkit_vehicle_factory.h",
        "protocol/bms_report_512.h",
        "protocol/brake_command_101.h",
        "protocol/brake_report_501.h",
        "protocol/gear_command_103.h",
        "protocol/gear_report_503.h",
        "protocol/park_command_104.h",
        "protocol/park_report_504.h",
        "protocol/steering_command_102.h",
        "protocol/steering_report_502.h",
        "protocol/throttle_command_100.h",
        "protocol/throttle_report_500.h",
        "protocol/ultr_sensor_1_507.h",
        "protocol/ultr_sensor_2_508.h",
        "protocol/ultr_sensor_3_509.h",
        "protocol/ultr_sensor_4_510.h",
        "protocol/ultr_sensor_5_511.h",
        "protocol/vcu_report_505.h",
        "protocol/vehicle_mode_command_105.h",
        "protocol/vin_resp1_514.h",
        "protocol/vin_resp2_515.h",
        "protocol/vin_resp3_516.h",
        "protocol/wheelspeed_report_506.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/devkit/proto:devkit_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libdevkit_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_devkit"],
)

apollo_cc_test(
    name = "devkit_vehicle_factory_test",
    size = "small",
    srcs = ["devkit_vehicle_factory_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "devkit_message_manager_test",
    size = "small",
    srcs = ["devkit_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "devkit_controller_test",
    size = "small",
    srcs = ["devkit_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "bms_report_512_test",
    size = "small",
    srcs = ["protocol/bms_report_512_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "brake_command_101_test",
    size = "small",
    srcs = ["protocol/brake_command_101_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "brake_report_501_test",
    size = "small",
    srcs = ["protocol/brake_report_501_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gear_command_103_test",
    size = "small",
    srcs = ["protocol/gear_command_103_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gear_report_503_test",
    size = "small",
    srcs = ["protocol/gear_report_503_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "park_command_104_test",
    size = "small",
    srcs = ["protocol/park_command_104_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "park_report_504_test",
    size = "small",
    srcs = ["protocol/park_report_504_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "steering_command_102_test",
    size = "small",
    srcs = ["protocol/steering_command_102_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "steering_report_502_test",
    size = "small",
    srcs = ["protocol/steering_report_502_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "throttle_command_100_test",
    size = "small",
    srcs = ["protocol/throttle_command_100_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "throttle_report_500_test",
    size = "small",
    srcs = ["protocol/throttle_report_500_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ultr_sensor_1_507_test",
    size = "small",
    srcs = ["protocol/ultr_sensor_1_507_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ultr_sensor_2_508_test",
    size = "small",
    srcs = ["protocol/ultr_sensor_2_508_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ultr_sensor_3_509_test",
    size = "small",
    srcs = ["protocol/ultr_sensor_3_509_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ultr_sensor_4_510_test",
    size = "small",
    srcs = ["protocol/ultr_sensor_4_510_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ultr_sensor_5_511_test",
    size = "small",
    srcs = ["protocol/ultr_sensor_5_511_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_report_505_test",
    size = "small",
    srcs = ["protocol/vcu_report_505_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "wheelspeed_report_506_test",
    size = "small",
    srcs = ["protocol/wheelspeed_report_506_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_devkit",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()

cpplint()
