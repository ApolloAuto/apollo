load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_neolix_edu_protocol",
    srcs = glob(
        ["*.cc"],
        exclude = ["*_test.cc"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ads_brake_command_46_test",
    size = "small",
    srcs = ["ads_brake_command_46_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_eps_command_56_test",
    size = "small",
    srcs = ["ads_eps_command_56_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_diagresp_718_test",
    size = "small",
    srcs = ["aeb_diagresp_718_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_systemstate_11_test",
    size = "small",
    srcs = ["aeb_systemstate_11_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pas_2nd_data_312_test",
    size = "small",
    srcs = ["pas_2nd_data_312_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_eps_report_57_test",
    size = "small",
    srcs = ["vcu_eps_report_57_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_vehicle_fault_response_201_test",
    size = "small",
    srcs = ["vcu_vehicle_fault_response_201_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_diagnosis_628_test",
    size = "small",
    srcs = ["ads_diagnosis_628_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_light_horn_command_310_test",
    size = "small",
    srcs = ["ads_light_horn_command_310_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_frontwheelspeed_353_test",
    size = "small",
    srcs = ["aeb_frontwheelspeed_353_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_wheelimpulse_355_test",
    size = "small",
    srcs = ["aeb_wheelimpulse_355_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_brake_report_47_test",
    size = "small",
    srcs = ["vcu_brake_report_47_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_nm_401_test",
    size = "small",
    srcs = ["vcu_nm_401_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_vehicle_info_response_502_test",
    size = "small",
    srcs = ["vcu_vehicle_info_response_502_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_drive_command_50_test",
    size = "small",
    srcs = ["ads_drive_command_50_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_diagnosis1_626_test",
    size = "small",
    srcs = ["aeb_diagnosis1_626_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aeb_rearwheelspeed_354_test",
    size = "small",
    srcs = ["aeb_rearwheelspeed_354_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pas_1st_data_311_test",
    size = "small",
    srcs = ["pas_1st_data_311_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_drive_report_52_test",
    size = "small",
    srcs = ["vcu_drive_report_52_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_powerstatus_214_test",
    size = "small",
    srcs = ["vcu_powerstatus_214_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vcu_vehicle_status_report_101_test",
    size = "small",
    srcs = ["vcu_vehicle_status_report_101_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
