load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_lincoln",
    srcs = [
        "lincoln_controller.cc",
        "lincoln_message_manager.cc",
        "lincoln_vehicle_factory.cc",
        "protocol/accel_6b.cc",
        "protocol/brake_60.cc",
        "protocol/brake_61.cc",
        "protocol/brakeinfo_74.cc",
        "protocol/fuellevel_72.cc",
        "protocol/gear_66.cc",
        "protocol/gear_67.cc",
        "protocol/gps_6d.cc",
        "protocol/gps_6e.cc",
        "protocol/gps_6f.cc",
        "protocol/gyro_6c.cc",
        "protocol/license_7e.cc",
        "protocol/misc_69.cc",
        "protocol/steering_64.cc",
        "protocol/steering_65.cc",
        "protocol/surround_73.cc",
        "protocol/throttle_62.cc",
        "protocol/throttle_63.cc",
        "protocol/throttleinfo_75.cc",
        "protocol/tirepressure_71.cc",
        "protocol/turnsignal_68.cc",
        "protocol/version_7f.cc",
        "protocol/wheelspeed_6a.cc",
    ],
    hdrs = [
        "lincoln_controller.h",
        "lincoln_message_manager.h",
        "lincoln_vehicle_factory.h",
        "protocol/accel_6b.h",
        "protocol/brake_60.h",
        "protocol/brake_61.h",
        "protocol/brakeinfo_74.h",
        "protocol/fuellevel_72.h",
        "protocol/gear_66.h",
        "protocol/gear_67.h",
        "protocol/gps_6d.h",
        "protocol/gps_6e.h",
        "protocol/gps_6f.h",
        "protocol/gyro_6c.h",
        "protocol/license_7e.h",
        "protocol/misc_69.h",
        "protocol/steering_64.h",
        "protocol/steering_65.h",
        "protocol/surround_73.h",
        "protocol/throttle_62.h",
        "protocol/throttle_63.h",
        "protocol/throttleinfo_75.h",
        "protocol/tirepressure_71.h",
        "protocol/turnsignal_68.h",
        "protocol/version_7f.h",
        "protocol/wheelspeed_6a.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/lincoln/proto:lincoln_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/kv_db",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "@com_google_googletest//:gtest",
    ],
)

apollo_component(
    name = "liblincoln_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_lincoln"],
)

apollo_cc_test(
    name = "lincoln_controller_test",
    size = "small",
    srcs = ["lincoln_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lincoln_message_manager_test",
    size = "small",
    srcs = ["lincoln_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lincoln_vehicle_factory_test",
    size = "small",
    srcs = ["lincoln_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_lincoln",
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
    name = "accel_6b_test",
    size = "small",
    srcs = ["protocol/accel_6b_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "brake_60_test",
    size = "small",
    srcs = ["protocol/brake_60_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "brake_61_test",
    size = "small",
    srcs = ["protocol/brake_61_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "brakeinfo_74_test",
    size = "small",
    srcs = ["protocol/brakeinfo_74_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fuellevel_72_test",
    size = "small",
    srcs = ["protocol/fuellevel_72_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gear_66_test",
    size = "small",
    srcs = ["protocol/gear_66_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gear_67_test",
    size = "small",
    srcs = ["protocol/gear_67_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gps_6d_test",
    size = "small",
    srcs = ["protocol/gps_6d_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gps_6e_test",
    size = "small",
    srcs = ["protocol/gps_6e_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gps_6f_test",
    size = "small",
    srcs = ["protocol/gps_6f_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "gyro_6c_test",
    size = "small",
    srcs = ["protocol/gyro_6c_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "misc_69_test",
    size = "small",
    srcs = ["protocol/misc_69_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "steering_64_test",
    size = "small",
    srcs = ["protocol/steering_64_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "steering_65_test",
    size = "small",
    srcs = ["protocol/steering_65_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "throttle_62_test",
    size = "small",
    srcs = ["protocol/throttle_62_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "throttle_63_test",
    size = "small",
    srcs = ["protocol/throttle_63_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "throttleinfo_75_test",
    size = "small",
    srcs = ["protocol/throttleinfo_75_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "tirepressure_71_test",
    size = "small",
    srcs = ["protocol/tirepressure_71_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "turnsignal_68_test",
    size = "small",
    srcs = ["protocol/turnsignal_68_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "version_7f_test",
    size = "small",
    srcs = ["protocol/version_7f_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "wheelspeed_6a_test",
    size = "small",
    srcs = ["protocol/wheelspeed_6a_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "surround_73_test",
    size = "small",
    srcs = ["protocol/surround_73_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_lincoln",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()
