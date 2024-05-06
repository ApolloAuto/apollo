load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_zhongyun",
    srcs = [
        "protocol/brake_control_a4.cc",
        "protocol/enable_state_feedback_c3.cc",
        "protocol/error_state_e1.cc",
        "protocol/gear_control_a1.cc",
        "protocol/parking_control_a5.cc",
        "protocol/steering_control_a2.cc",
        "protocol/torque_control_a3.cc",
        "protocol/vehicle_state_feedback_2_c4.cc",
        "protocol/vehicle_state_feedback_c1.cc",
        "zhongyun_controller.cc",
        "zhongyun_message_manager.cc",
        "zhongyun_vehicle_factory.cc",
    ],
    hdrs = [
        "protocol/brake_control_a4.h",
        "protocol/enable_state_feedback_c3.h",
        "protocol/error_state_e1.h",
        "protocol/gear_control_a1.h",
        "protocol/parking_control_a5.h",
        "protocol/steering_control_a2.h",
        "protocol/torque_control_a3.h",
        "protocol/vehicle_state_feedback_2_c4.h",
        "protocol/vehicle_state_feedback_c1.h",
        "zhongyun_controller.h",
        "zhongyun_message_manager.h",
        "zhongyun_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/zhongyun/proto:zhongyun_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)


apollo_component(
    name = "libzhongyun_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_zhongyun"],
)

apollo_cc_test(
    name = "zhongyun_message_manager_test",
    size = "small",
    srcs = ["zhongyun_message_manager_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "zhongyun_vehicle_factory_test",
    size = "small",
    srcs = ["zhongyun_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "zhongyun_controller_test",
    size = "small",
    srcs = ["zhongyun_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "//modules/common/util:common_util",
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
    name = "brake_control_a4_test",
    size = "small",
    srcs = ["protocol/brake_control_a4_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "enable_state_feedback_c3_test",
    size = "small",
    srcs = ["protocol/enable_state_feedback_c3_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "error_state_e1_test",
    size = "small",
    srcs = ["protocol/error_state_e1_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gear_control_a1_test",
    size = "small",
    srcs = ["protocol/gear_control_a1_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "parking_control_a5_test",
    size = "small",
    srcs = ["protocol/parking_control_a5_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "steering_control_a2_test",
    size = "small",
    srcs = ["protocol/steering_control_a2_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "torque_control_a3_test",
    size = "small",
    srcs = ["protocol/torque_control_a3_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vehicle_state_feedback_2_c4_test",
    size = "small",
    srcs = ["protocol/vehicle_state_feedback_2_c4_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "vehicle_state_feedback_c1_test",
    size = "small",
    srcs = ["protocol/vehicle_state_feedback_c1_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_zhongyun",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()