load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_ge3",
    srcs = [
        "ge3_controller.cc",
        "ge3_message_manager.cc",
        "ge3_vehicle_factory.cc",
        "protocol/pc_bcm_201.cc",
        "protocol/pc_bcs_202.cc",
        "protocol/pc_epb_203.cc",
        "protocol/pc_eps_204.cc",
        "protocol/pc_vcu_205.cc",
        "protocol/scu_1_301.cc",
        "protocol/scu_2_302.cc",
        "protocol/scu_3_303.cc",
        "protocol/scu_bcm_304.cc",
        "protocol/scu_bcs_1_306.cc",
        "protocol/scu_bcs_2_307.cc",
        "protocol/scu_bcs_3_308.cc",
        "protocol/scu_epb_310.cc",
        "protocol/scu_eps_311.cc",
        "protocol/scu_vcu_1_312.cc",
        "protocol/scu_vcu_2_313.cc",
    ],
    hdrs = [
        "ge3_controller.h",
        "ge3_message_manager.h",
        "ge3_vehicle_factory.h",
        "protocol/pc_bcm_201.h",
        "protocol/pc_bcs_202.h",
        "protocol/pc_epb_203.h",
        "protocol/pc_eps_204.h",
        "protocol/pc_vcu_205.h",
        "protocol/scu_1_301.h",
        "protocol/scu_2_302.h",
        "protocol/scu_3_303.h",
        "protocol/scu_bcm_304.h",
        "protocol/scu_bcs_1_306.h",
        "protocol/scu_bcs_2_307.h",
        "protocol/scu_bcs_3_308.h",
        "protocol/scu_epb_310.h",
        "protocol/scu_eps_311.h",
        "protocol/scu_vcu_1_312.h",
        "protocol/scu_vcu_2_313.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/ge3/proto:ge3_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libge3_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_ge3"],
)

apollo_cc_test(
    name = "ge3_vehicle_factory_test",
    size = "small",
    srcs = ["ge3_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ge3_message_manager_test",
    size = "small",
    srcs = ["ge3_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ge3_controller_test",
    size = "small",
    srcs = ["ge3_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
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
    name = "pc_bcm_201_test",
    size = "small",
    srcs = ["protocol/pc_bcm_201_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "pc_bcs_202_test",
    size = "small",
    srcs = ["protocol/pc_bcs_202_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "pc_epb_203_test",
    size = "small",
    srcs = ["protocol/pc_epb_203_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "pc_eps_204_test",
    size = "small",
    srcs = ["protocol/pc_eps_204_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "pc_vcu_205_test",
    size = "small",
    srcs = ["protocol/pc_vcu_205_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_1_301_test",
    size = "small",
    srcs = ["protocol/scu_1_301_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_2_302_test",
    size = "small",
    srcs = ["protocol/scu_2_302_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_3_303_test",
    size = "small",
    srcs = ["protocol/scu_3_303_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_bcm_304_test",
    size = "small",
    srcs = ["protocol/scu_bcm_304_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_bcs_1_306_test",
    size = "small",
    srcs = ["protocol/scu_bcs_1_306_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_bcs_2_307_test",
    size = "small",
    srcs = ["protocol/scu_bcs_2_307_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_bcs_3_308_test",
    size = "small",
    srcs = ["protocol/scu_bcs_3_308_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_epb_310_test",
    size = "small",
    srcs = ["protocol/scu_epb_310_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_eps_311_test",
    size = "small",
    srcs = ["protocol/scu_eps_311_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_vcu_1_312_test",
    size = "small",
    srcs = ["protocol/scu_vcu_1_312_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "scu_vcu_2_313_test",
    size = "small",
    srcs = ["protocol/scu_vcu_2_313_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ge3",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()

