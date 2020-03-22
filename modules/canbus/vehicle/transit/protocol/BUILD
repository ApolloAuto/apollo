load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_transit_protocol",
    deps = [
        ":adc_auxiliarycontrol_110",
        ":adc_motioncontrol1_10",
        ":adc_motioncontrollimits1_12",
        ":llc_auxiliaryfeedback_120",
        ":llc_diag_brakecontrol_721",
        ":llc_diag_fault_620",
        ":llc_diag_steeringcontrol_722",
        ":llc_motioncommandfeedback1_22",
        ":llc_motionfeedback1_20",
        ":llc_motionfeedback2_21",
        ":llc_vehiclelimits_24",
        ":llc_vehiclestatus_25",
    ],
)

cc_library(
    name = "adc_auxiliarycontrol_110",
    srcs = ["adc_auxiliarycontrol_110.cc"],
    hdrs = ["adc_auxiliarycontrol_110.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Auxiliarycontrol_110_test",
    size = "small",
    srcs = ["adc_auxiliarycontrol_110_test.cc"],
    deps = [
        ":adc_auxiliarycontrol_110",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "adc_motioncontrol1_10",
    srcs = ["adc_motioncontrol1_10.cc"],
    hdrs = ["adc_motioncontrol1_10.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "adc_motioncontrol1_10_test",
    size = "small",
    srcs = ["adc_motioncontrol1_10_test.cc"],
    deps = [
        ":adc_motioncontrol1_10",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "adc_motioncontrollimits1_12",
    srcs = ["adc_motioncontrollimits1_12.cc"],
    hdrs = ["adc_motioncontrollimits1_12.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Motioncontrollimits1_12",
    size = "small",
    srcs = ["adc_motioncontrollimits1_12_test.cc"],
    deps = [
        ":adc_motioncontrollimits1_12",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_auxiliaryfeedback_120",
    srcs = ["llc_auxiliaryfeedback_120.cc"],
    hdrs = ["llc_auxiliaryfeedback_120.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "llc_auxiliaryfeedback_120_test",
    size = "small",
    srcs = ["llc_auxiliaryfeedback_120_test.cc"],
    deps = [
        ":llc_auxiliaryfeedback_120",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_diag_brakecontrol_721",
    srcs = ["llc_diag_brakecontrol_721.cc"],
    hdrs = ["llc_diag_brakecontrol_721.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "llc_diag_brakecontrol_721_test",
    size = "small",
    srcs = ["llc_diag_brakecontrol_721_test.cc"],
    deps = [
        ":llc_diag_brakecontrol_721",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_diag_fault_620",
    srcs = ["llc_diag_fault_620.cc"],
    hdrs = ["llc_diag_fault_620.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Diag_fault_620_test",
    size = "small",
    srcs = ["llc_diag_fault_620_test.cc"],
    deps = [
        ":llc_diag_fault_620",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_diag_steeringcontrol_722",
    srcs = ["llc_diag_steeringcontrol_722.cc"],
    hdrs = ["llc_diag_steeringcontrol_722.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "llc_diag_steeringcontrol_722_test",
    size = "small",
    srcs = ["llc_diag_steeringcontrol_722_test.cc"],
    deps = [
        ":llc_diag_steeringcontrol_722",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_motioncommandfeedback1_22",
    srcs = ["llc_motioncommandfeedback1_22.cc"],
    hdrs = ["llc_motioncommandfeedback1_22.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Motioncommandfeedback1_22_test",
    size = "small",
    srcs = ["llc_motioncommandfeedback1_22_test.cc"],
    deps = [
        ":llc_motioncommandfeedback1_22",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_motionfeedback1_20",
    srcs = ["llc_motionfeedback1_20.cc"],
    hdrs = ["llc_motionfeedback1_20.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Motionfeedback1_20_test",
    size = "small",
    srcs = ["llc_motionfeedback1_20_test.cc"],
    deps = [
        ":llc_motionfeedback1_20",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_motionfeedback2_21",
    srcs = ["llc_motionfeedback2_21.cc"],
    hdrs = ["llc_motionfeedback2_21.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "llc_motionfeedback2_21_test",
    size = "small",
    srcs = ["llc_motionfeedback2_21_test.cc"],
    deps = [
        ":llc_motionfeedback2_21",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_vehiclelimits_24",
    srcs = ["llc_vehiclelimits_24.cc"],
    hdrs = ["llc_vehiclelimits_24.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "Vehiclelimits_24_test",
    size = "small",
    srcs = ["llc_vehiclelimits_24_test.cc"],
    deps = [
        ":llc_vehiclelimits_24",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "llc_vehiclestatus_25",
    srcs = ["llc_vehiclestatus_25.cc"],
    hdrs = ["llc_vehiclestatus_25.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "llc_vehiclestatus_25_test",
    size = "small",
    srcs = ["llc_vehiclestatus_25_test.cc"],
    deps = [
        ":llc_vehiclestatus_25",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
