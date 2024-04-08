load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_transit",
    srcs = [
        "protocol/adc_auxiliarycontrol_110.cc",
        "protocol/adc_motioncontrol1_10.cc",
        "protocol/adc_motioncontrollimits1_12.cc",
        "protocol/llc_auxiliaryfeedback_120.cc",
        "protocol/llc_diag_brakecontrol_721.cc",
        "protocol/llc_diag_fault_620.cc",
        "protocol/llc_diag_steeringcontrol_722.cc",
        "protocol/llc_motioncommandfeedback1_22.cc",
        "protocol/llc_motionfeedback1_20.cc",
        "protocol/llc_motionfeedback2_21.cc",
        "protocol/llc_vehiclelimits_24.cc",
        "protocol/llc_vehiclestatus_25.cc",
        "transit_controller.cc",
        "transit_message_manager.cc",
        "transit_vehicle_factory.cc",
    ],
    hdrs = [
        "protocol/adc_auxiliarycontrol_110.h",
        "protocol/adc_motioncontrol1_10.h",
        "protocol/adc_motioncontrollimits1_12.h",
        "protocol/llc_auxiliaryfeedback_120.h",
        "protocol/llc_diag_brakecontrol_721.h",
        "protocol/llc_diag_fault_620.h",
        "protocol/llc_diag_steeringcontrol_722.h",
        "protocol/llc_motioncommandfeedback1_22.h",
        "protocol/llc_motionfeedback1_20.h",
        "protocol/llc_motionfeedback2_21.h",
        "protocol/llc_vehiclelimits_24.h",
        "protocol/llc_vehiclestatus_25.h",
        "transit_controller.h",
        "transit_message_manager.h",
        "transit_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/transit/proto:transit_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/kv_db",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "@com_google_googletest//:gtest",
    ],
)


apollo_component(
    name = "libtransit_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_transit"],
)

apollo_cc_test(
    name = "transit_vehicle_factory_test",
    size = "small",
    srcs = ["transit_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "transit_controller_test",
    size = "small",
    srcs = ["transit_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_transit",
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
    name = "Auxiliarycontrol_110_test",
    size = "small",
    srcs = ["protocol/adc_auxiliarycontrol_110_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "adc_motioncontrol1_10_test",
    size = "small",
    srcs = ["protocol/adc_motioncontrol1_10_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "Motioncontrollimits1_12",
    size = "small",
    srcs = ["protocol/adc_motioncontrollimits1_12_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "llc_auxiliaryfeedback_120_test",
    size = "small",
    srcs = ["protocol/llc_auxiliaryfeedback_120_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "llc_diag_brakecontrol_721_test",
    size = "small",
    srcs = ["protocol/llc_diag_brakecontrol_721_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "Diag_fault_620_test",
    size = "small",
    srcs = ["protocol/llc_diag_fault_620_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "llc_diag_steeringcontrol_722_test",
    size = "small",
    srcs = ["protocol/llc_diag_steeringcontrol_722_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "Motioncommandfeedback1_22_test",
    size = "small",
    srcs = ["protocol/llc_motioncommandfeedback1_22_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "Motionfeedback1_20_test",
    size = "small",
    srcs = ["protocol/llc_motionfeedback1_20_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "llc_motionfeedback2_21_test",
    size = "small",
    srcs = ["protocol/llc_motionfeedback2_21_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "Vehiclelimits_24_test",
    size = "small",
    srcs = ["protocol/llc_vehiclelimits_24_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "llc_vehiclestatus_25_test",
    size = "small",
    srcs = ["protocol/llc_vehiclestatus_25_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_transit",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()
