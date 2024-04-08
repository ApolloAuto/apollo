load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_gem",
    srcs = [
        "gem_controller.cc",
        "gem_message_manager.cc",
        "gem_vehicle_factory.cc",
        "protocol/accel_cmd_67.cc",
        "protocol/accel_rpt_68.cc",
        "protocol/brake_cmd_6b.cc",
        "protocol/brake_motor_rpt_1_70.cc",
        "protocol/brake_motor_rpt_2_71.cc",
        "protocol/brake_motor_rpt_3_72.cc",
        "protocol/brake_rpt_6c.cc",
        "protocol/date_time_rpt_83.cc",
        "protocol/global_cmd_69.cc",
        "protocol/global_rpt_6a.cc",
        "protocol/headlight_cmd_76.cc",
        "protocol/headlight_rpt_77.cc",
        "protocol/horn_cmd_78.cc",
        "protocol/horn_rpt_79.cc",
        "protocol/lat_lon_heading_rpt_82.cc",
        "protocol/parking_brake_status_rpt_80.cc",
        "protocol/shift_cmd_65.cc",
        "protocol/shift_rpt_66.cc",
        "protocol/steering_cmd_6d.cc",
        "protocol/steering_motor_rpt_1_73.cc",
        "protocol/steering_motor_rpt_2_74.cc",
        "protocol/steering_motor_rpt_3_75.cc",
        "protocol/steering_rpt_1_6e.cc",
        "protocol/turn_cmd_63.cc",
        "protocol/turn_rpt_64.cc",
        "protocol/vehicle_speed_rpt_6f.cc",
        "protocol/wheel_speed_rpt_7a.cc",
        "protocol/wiper_cmd_90.cc",
        "protocol/wiper_rpt_91.cc",
        "protocol/yaw_rate_rpt_81.cc",
    ],
    hdrs = [
        "gem_controller.h",
        "gem_message_manager.h",
        "gem_vehicle_factory.h",
        "protocol/accel_cmd_67.h",
        "protocol/accel_rpt_68.h",
        "protocol/brake_cmd_6b.h",
        "protocol/brake_motor_rpt_1_70.h",
        "protocol/brake_motor_rpt_2_71.h",
        "protocol/brake_motor_rpt_3_72.h",
        "protocol/brake_rpt_6c.h",
        "protocol/date_time_rpt_83.h",
        "protocol/global_cmd_69.h",
        "protocol/global_rpt_6a.h",
        "protocol/headlight_cmd_76.h",
        "protocol/headlight_rpt_77.h",
        "protocol/horn_cmd_78.h",
        "protocol/horn_rpt_79.h",
        "protocol/lat_lon_heading_rpt_82.h",
        "protocol/parking_brake_status_rpt_80.h",
        "protocol/shift_cmd_65.h",
        "protocol/shift_rpt_66.h",
        "protocol/steering_cmd_6d.h",
        "protocol/steering_motor_rpt_1_73.h",
        "protocol/steering_motor_rpt_2_74.h",
        "protocol/steering_motor_rpt_3_75.h",
        "protocol/steering_rpt_1_6e.h",
        "protocol/turn_cmd_63.h",
        "protocol/turn_rpt_64.h",
        "protocol/vehicle_speed_rpt_6f.h",
        "protocol/wheel_speed_rpt_7a.h",
        "protocol/wiper_cmd_90.h",
        "protocol/wiper_rpt_91.h",
        "protocol/yaw_rate_rpt_81.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/gem/proto:gem_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libgem_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_gem"],
)

apollo_cc_test(
    name = "gem_vehicle_factory_test",
    size = "small",
    srcs = ["gem_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gem_message_manager_test",
    size = "small",
    srcs = ["gem_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gem_controller_test",
    size = "small",
    srcs = ["gem_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "accel_rpt_68_test",
    size = "small",
    srcs = ["protocol/accel_rpt_68_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "brake_rpt_6c_test",
    size = "small",
    srcs = ["protocol/brake_rpt_6c_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "wheel_speed_rpt_7a_test",
    size = "small",
    srcs = ["protocol/wheel_speed_rpt_7a_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "yaw_rate_rpt_81_test",
    size = "small",
    srcs = ["protocol/yaw_rate_rpt_81_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "wiper_rpt_91_test",
    size = "small",
    srcs = ["protocol/wiper_rpt_91_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "brake_motor_rpt_1_70_test",
    size = "small",
    srcs = ["protocol/brake_motor_rpt_1_70_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "brake_motor_rpt_2_71_test",
    size = "small",
    srcs = ["protocol/brake_motor_rpt_2_71_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "brake_motor_rpt_3_72_test",
    size = "small",
    srcs = ["protocol/brake_motor_rpt_3_72_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "date_time_rpt_83_test",
    size = "small",
    srcs = ["protocol/date_time_rpt_83_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "global_rpt_6a_test",
    size = "small",
    srcs = ["protocol/global_rpt_6a_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "lat_lon_heading_rpt_82_test",
    size = "small",
    srcs = ["protocol/lat_lon_heading_rpt_82_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "steering_motor_rpt_1_73_test",
    size = "small",
    srcs = ["protocol/steering_motor_rpt_1_73_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "steering_motor_rpt_2_74_test",
    size = "small",
    srcs = ["protocol/steering_motor_rpt_2_74_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "steering_motor_rpt_3_75_test",
    size = "small",
    srcs = ["protocol/steering_motor_rpt_3_75_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_cc_test(
    name = "steering_rpt_1_6e_test",
    size = "small",
    srcs = ["protocol/steering_rpt_1_6e_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "accel_cmd_67_test",
    size = "small",
    srcs = ["protocol/accel_cmd_67_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_gem",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()
