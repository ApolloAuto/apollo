load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_wey",
    srcs = [
        "protocol/ads1_111.cc",
        "protocol/ads3_38e.cc",
        "protocol/ads_eps_113.cc",
        "protocol/ads_req_vin_390.cc",
        "protocol/ads_shifter_115.cc",
        "protocol/fail_241.cc",
        "protocol/fbs1_243.cc",
        "protocol/fbs2_240.cc",
        "protocol/fbs3_237.cc",
        "protocol/fbs4_235.cc",
        "protocol/status_310.cc",
        "protocol/vin_resp1_391.cc",
        "protocol/vin_resp2_392.cc",
        "protocol/vin_resp3_393.cc",
        "wey_controller.cc",
        "wey_message_manager.cc",
        "wey_vehicle_factory.cc",
    ],
    hdrs = [
        "protocol/ads1_111.h",
        "protocol/ads3_38e.h",
        "protocol/ads_eps_113.h",
        "protocol/ads_req_vin_390.h",
        "protocol/ads_shifter_115.h",
        "protocol/fail_241.h",
        "protocol/fbs1_243.h",
        "protocol/fbs2_240.h",
        "protocol/fbs3_237.h",
        "protocol/fbs4_235.h",
        "protocol/status_310.h",
        "protocol/vin_resp1_391.h",
        "protocol/vin_resp2_392.h",
        "protocol/vin_resp3_393.h",
        "wey_controller.h",
        "wey_message_manager.h",
        "wey_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/wey/proto:wey_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)


apollo_component(
    name = "libwey_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_wey"],
)

apollo_cc_test(
    name = "wey_vehicle_factory_test",
    size = "small",
    srcs = ["wey_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "wey_message_manager_test",
    size = "small",
    srcs = ["wey_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "wey_controller_test",
    size = "small",
    srcs = ["wey_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_wey",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ads1_111_test",
    size = "small",
    srcs = ["protocol/ads1_111_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads3_38e_test",
    size = "small",
    srcs = ["protocol/ads3_38e_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads_eps_113_test",
    size = "small",
    srcs = ["protocol/ads_eps_113_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads_req_vin_390_test",
    size = "small",
    srcs = ["protocol/ads_req_vin_390_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ads_shifter_115_test",
    size = "small",
    srcs = ["protocol/ads_shifter_115_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fail_241_test",
    size = "small",
    srcs = ["protocol/fail_241_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fbs1_243_test",
    size = "small",
    srcs = ["protocol/fbs1_243_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fbs2_240_test",
    size = "small",
    srcs = ["protocol/fbs2_240_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fbs3_237_test",
    size = "small",
    srcs = ["protocol/fbs3_237_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "fbs4_235_test",
    size = "small",
    srcs = ["protocol/fbs4_235_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "status_310_test",
    size = "small",
    srcs = ["protocol/status_310_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "vin_resp1_391_test",
    size = "small",
    srcs = ["protocol/vin_resp1_391_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "vin_resp2_392_test",
    size = "small",
    srcs = ["protocol/vin_resp2_392_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "vin_resp3_393_test",
    size = "small",
    srcs = ["protocol/vin_resp3_393_test.cc"],
    deps = [
        "apollo_canbus_vehicle_wey",
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
