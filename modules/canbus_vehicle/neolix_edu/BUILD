load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_neolix_edu",
    srcs = [
        "neolix_edu_controller.cc",
        "neolix_edu_message_manager.cc",
        "neolix_edu_vehicle_factory.cc",
        "protocol/ads_brake_command_46.cc",
        "protocol/ads_diagnosis_628.cc",
        "protocol/ads_drive_command_50.cc",
        "protocol/ads_eps_command_56.cc",
        "protocol/ads_light_horn_command_310.cc",
        "protocol/aeb_diagnosis1_626.cc",
        "protocol/aeb_diagresp_718.cc",
        "protocol/aeb_frontwheelspeed_353.cc",
        "protocol/aeb_rearwheelspeed_354.cc",
        "protocol/aeb_systemstate_11.cc",
        "protocol/aeb_wheelimpulse_355.cc",
        "protocol/pas_1st_data_311.cc",
        "protocol/pas_2nd_data_312.cc",
        "protocol/vcu_brake_report_47.cc",
        "protocol/vcu_drive_report_52.cc",
        "protocol/vcu_eps_report_57.cc",
        "protocol/vcu_nm_401.cc",
        "protocol/vcu_powerstatus_214.cc",
        "protocol/vcu_vehicle_fault_response_201.cc",
        "protocol/vcu_vehicle_info_response_502.cc",
        "protocol/vcu_vehicle_status_report_101.cc",
    ],
    hdrs = [
        "neolix_edu_controller.h",
        "neolix_edu_message_manager.h",
        "neolix_edu_vehicle_factory.h",
        "protocol/ads_brake_command_46.h",
        "protocol/ads_diagnosis_628.h",
        "protocol/ads_drive_command_50.h",
        "protocol/ads_eps_command_56.h",
        "protocol/ads_light_horn_command_310.h",
        "protocol/aeb_diagnosis1_626.h",
        "protocol/aeb_diagresp_718.h",
        "protocol/aeb_frontwheelspeed_353.h",
        "protocol/aeb_rearwheelspeed_354.h",
        "protocol/aeb_systemstate_11.h",
        "protocol/aeb_wheelimpulse_355.h",
        "protocol/pas_1st_data_311.h",
        "protocol/pas_2nd_data_312.h",
        "protocol/vcu_brake_report_47.h",
        "protocol/vcu_drive_report_52.h",
        "protocol/vcu_eps_report_57.h",
        "protocol/vcu_nm_401.h",
        "protocol/vcu_powerstatus_214.h",
        "protocol/vcu_vehicle_fault_response_201.h",
        "protocol/vcu_vehicle_info_response_502.h",
        "protocol/vcu_vehicle_status_report_101.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/neolix_edu/proto:neolix_edu_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libneolix_edu_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_neolix_edu"],
)

apollo_cc_test(
    name = "neolix_edu_controller_test",
    size = "small",
    srcs = ["neolix_edu_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "neolix_edu_message_manager_test",
    size = "small",
    srcs = ["neolix_edu_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "neolix_edu_vehicle_factory_test",
    size = "small",
    srcs = ["neolix_edu_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ads_brake_command_46_test",
    size = "small",
    srcs = ["protocol/ads_brake_command_46_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ads_eps_command_56_test",
    size = "small",
    srcs = ["protocol/ads_eps_command_56_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aeb_diagresp_718_test",
    size = "small",
    srcs = ["protocol/aeb_diagresp_718_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aeb_systemstate_11_test",
    size = "small",
    srcs = ["protocol/aeb_systemstate_11_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pas_2nd_data_312_test",
    size = "small",
    srcs = ["protocol/pas_2nd_data_312_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_eps_report_57_test",
    size = "small",
    srcs = ["protocol/vcu_eps_report_57_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_vehicle_fault_response_201_test",
    size = "small",
    srcs = ["protocol/vcu_vehicle_fault_response_201_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads_diagnosis_628_test",
    size = "small",
    srcs = ["protocol/ads_diagnosis_628_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ads_light_horn_command_310_test",
    size = "small",
    srcs = ["protocol/ads_light_horn_command_310_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "aeb_frontwheelspeed_353_test",
    size = "small",
    srcs = ["protocol/aeb_frontwheelspeed_353_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "aeb_wheelimpulse_355_test",
    size = "small",
    srcs = ["protocol/aeb_wheelimpulse_355_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_brake_report_47_test",
    size = "small",
    srcs = ["protocol/vcu_brake_report_47_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_nm_401_test",
    size = "small",
    srcs = ["protocol/vcu_nm_401_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "vcu_vehicle_info_response_502_test",
    size = "small",
    srcs = ["protocol/vcu_vehicle_info_response_502_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads_drive_command_50_test",
    size = "small",
    srcs = ["protocol/ads_drive_command_50_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aeb_diagnosis1_626_test",
    size = "small",
    srcs = ["protocol/aeb_diagnosis1_626_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aeb_rearwheelspeed_354_test",
    size = "small",
    srcs = ["protocol/aeb_rearwheelspeed_354_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "pas_1st_data_311_test",
    size = "small",
    srcs = ["protocol/pas_1st_data_311_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_drive_report_52_test",
    size = "small",
    srcs = ["protocol/vcu_drive_report_52_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_powerstatus_214_test",
    size = "small",
    srcs = ["protocol/vcu_powerstatus_214_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vcu_vehicle_status_report_101_test",
    size = "small",
    srcs = ["protocol/vcu_vehicle_status_report_101_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_neolix_edu",
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

apollo_package()

cpplint()
