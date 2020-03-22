load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_ge3_protocol_pc_bcm_201",
    srcs = [
        "pc_bcm_201.cc",
        "pc_bcs_202.cc",
        "pc_epb_203.cc",
        "pc_eps_204.cc",
        "pc_vcu_205.cc",
        "scu_1_301.cc",
        "scu_2_302.cc",
        "scu_3_303.cc",
        "scu_bcm_304.cc",
        "scu_bcs_1_306.cc",
        "scu_bcs_2_307.cc",
        "scu_bcs_3_308.cc",
        "scu_epb_310.cc",
        "scu_eps_311.cc",
        "scu_vcu_1_312.cc",
        "scu_vcu_2_313.cc",
    ],
    hdrs = [
        "pc_bcm_201.h",
        "pc_bcs_202.h",
        "pc_epb_203.h",
        "pc_eps_204.h",
        "pc_vcu_205.h",
        "scu_1_301.h",
        "scu_2_302.h",
        "scu_3_303.h",
        "scu_bcm_304.h",
        "scu_bcs_1_306.h",
        "scu_bcs_2_307.h",
        "scu_bcs_3_308.h",
        "scu_epb_310.h",
        "scu_eps_311.h",
        "scu_vcu_1_312.h",
        "scu_vcu_2_313.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "pc_bcm_201_test",
    size = "small",
    srcs = ["pc_bcm_201_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pc_bcs_202_test",
    size = "small",
    srcs = ["pc_bcs_202_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pc_epb_203_test",
    size = "small",
    srcs = ["pc_epb_203_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pc_eps_204_test",
    size = "small",
    srcs = ["pc_eps_204_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pc_vcu_205_test",
    size = "small",
    srcs = ["pc_vcu_205_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_1_301_test",
    size = "small",
    srcs = ["scu_1_301_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_2_302_test",
    size = "small",
    srcs = ["scu_2_302_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_3_303_test",
    size = "small",
    srcs = ["scu_3_303_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_bcm_304_test",
    size = "small",
    srcs = ["scu_bcm_304_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_bcs_1_306_test",
    size = "small",
    srcs = ["scu_bcs_1_306_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_bcs_2_307_test",
    size = "small",
    srcs = ["scu_bcs_2_307_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_bcs_3_308_test",
    size = "small",
    srcs = ["scu_bcs_3_308_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_epb_310_test",
    size = "small",
    srcs = ["scu_epb_310_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_eps_311_test",
    size = "small",
    srcs = ["scu_eps_311_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_vcu_1_312_test",
    size = "small",
    srcs = ["scu_vcu_1_312_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scu_vcu_2_313_test",
    size = "small",
    srcs = ["scu_vcu_2_313_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
