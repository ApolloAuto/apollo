load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_neolix_edu_protocol",
    srcs = [
        "ads_brake_command_46.cc",
        "ads_diagnosis_628.cc",
        "ads_drive_command_50.cc",
        "ads_eps_command_56.cc",
        "ads_light_horn_command_310.cc",
        "aeb_diagnosis1_626.cc",
        "aeb_diagresp_718.cc",
        "aeb_frontwheelspeed_353.cc",
        "aeb_rearwheelspeed_354.cc",
        "aeb_systemstate_11.cc",
        "aeb_wheelimpulse_355.cc",
        "pas_1st_data_311.cc",
        "pas_2nd_data_312.cc",
        "vcu_brake_report_47.cc",
        "vcu_drive_report_52.cc",
        "vcu_eps_report_57.cc",
        "vcu_nm_401.cc",
        "vcu_powerstatus_214.cc",
        "vcu_vehicle_fault_response_201.cc",
        "vcu_vehicle_info_response_502.cc",
        "vcu_vehicle_status_report_101.cc",
    ],
    hdrs = [
        "ads_brake_command_46.h",
        "ads_diagnosis_628.h",
        "ads_drive_command_50.h",
        "ads_eps_command_56.h",
        "ads_light_horn_command_310.h",
        "aeb_diagnosis1_626.h",
        "aeb_diagresp_718.h",
        "aeb_frontwheelspeed_353.h",
        "aeb_rearwheelspeed_354.h",
        "aeb_systemstate_11.h",
        "aeb_wheelimpulse_355.h",
        "pas_1st_data_311.h",
        "pas_2nd_data_312.h",
        "vcu_brake_report_47.h",
        "vcu_drive_report_52.h",
        "vcu_eps_report_57.h",
        "vcu_nm_401.h",
        "vcu_powerstatus_214.h",
        "vcu_vehicle_fault_response_201.h",
        "vcu_vehicle_info_response_502.h",
        "vcu_vehicle_status_report_101.h",
    ],
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
