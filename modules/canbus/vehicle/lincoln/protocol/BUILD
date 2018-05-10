load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_lincoln_protocol",
    srcs = [
        "accel_6b.cc",
        "brake_60.cc",
        "brake_61.cc",
        "brakeinfo_74.cc",
        "fuellevel_72.cc",
        "gear_66.cc",
        "gear_67.cc",
        "gps_6d.cc",
        "gps_6e.cc",
        "gps_6f.cc",
        "gyro_6c.cc",
        "license_7e.cc",
        "misc_69.cc",
        "steering_64.cc",
        "steering_65.cc",
        "surround_73.cc",
        "throttle_62.cc",
        "throttle_63.cc",
        "throttleinfo_75.cc",
        "tirepressure_71.cc",
        "turnsignal_68.cc",
        "version_7f.cc",
        "wheelspeed_6a.cc",
    ],
    hdrs = [
        "accel_6b.h",
        "brake_60.h",
        "brake_61.h",
        "brakeinfo_74.h",
        "fuellevel_72.h",
        "gear_66.h",
        "gear_67.h",
        "gps_6d.h",
        "gps_6e.h",
        "gps_6f.h",
        "gyro_6c.h",
        "license_7e.h",
        "misc_69.h",
        "steering_64.h",
        "steering_65.h",
        "surround_73.h",
        "throttle_62.h",
        "throttle_63.h",
        "throttleinfo_75.h",
        "tirepressure_71.h",
        "turnsignal_68.h",
        "version_7f.h",
        "wheelspeed_6a.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "accel_6b_test",
    size = "small",
    srcs = [
        "accel_6b_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "brake_60_test",
    size = "small",
    srcs = [
        "brake_60_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "brake_61_test",
    size = "small",
    srcs = [
        "brake_61_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "brakeinfo_74_test",
    size = "small",
    srcs = [
        "brakeinfo_74_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "fuellevel_72_test",
    size = "small",
    srcs = [
        "fuellevel_72_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gear_66_test",
    size = "small",
    srcs = [
        "gear_66_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gear_67_test",
    size = "small",
    srcs = [
        "gear_67_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gps_6d_test",
    size = "small",
    srcs = [
        "gps_6d_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gps_6e_test",
    size = "small",
    srcs = [
        "gps_6e_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gps_6f_test",
    size = "small",
    srcs = [
        "gps_6f_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gyro_6c_test",
    size = "small",
    srcs = [
        "gyro_6c_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "misc_69_test",
    size = "small",
    srcs = [
        "misc_69_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "steering_64_test",
    size = "small",
    srcs = [
        "steering_64_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "steering_65_test",
    size = "small",
    srcs = [
        "steering_65_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttle_62_test",
    size = "small",
    srcs = [
        "throttle_62_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttle_63_test",
    size = "small",
    srcs = [
        "throttle_63_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttleinfo_75_test",
    size = "small",
    srcs = [
        "throttleinfo_75_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "tirepressure_71_test",
    size = "small",
    srcs = [
        "tirepressure_71_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "turnsignal_68_test",
    size = "small",
    srcs = [
        "turnsignal_68_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "version_7f_test",
    size = "small",
    srcs = [
        "version_7f_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "wheelspeed_6a_test",
    size = "small",
    srcs = [
        "wheelspeed_6a_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "surround_73_test",
    size = "small",
    srcs = [
        "surround_73_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "@gtest//:main",
    ],
)

cpplint()
