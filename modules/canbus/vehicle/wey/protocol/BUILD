load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_wey_protocol",
    srcs = [
        "ads1_111.cc",
        "ads3_38e.cc",
        "ads_eps_113.cc",
        "ads_req_vin_390.cc",
        "ads_shifter_115.cc",
        "fail_241.cc",
        "fbs1_243.cc",
        "fbs2_240.cc",
        "fbs3_237.cc",
        "fbs4_235.cc",
        "status_310.cc",
        "vin_resp1_391.cc",
        "vin_resp2_392.cc",
        "vin_resp3_393.cc",
    ],
    hdrs = [
        "ads1_111.h",
        "ads3_38e.h",
        "ads_eps_113.h",
        "ads_req_vin_390.h",
        "ads_shifter_115.h",
        "fail_241.h",
        "fbs1_243.h",
        "fbs2_240.h",
        "fbs3_237.h",
        "fbs4_235.h",
        "status_310.h",
        "vin_resp1_391.h",
        "vin_resp2_392.h",
        "vin_resp3_393.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ads1_111_test",
    size = "small",
    srcs = ["ads1_111_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads3_38e_test",
    size = "small",
    srcs = ["ads3_38e_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_eps_113_test",
    size = "small",
    srcs = ["ads_eps_113_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_req_vin_390_test",
    size = "small",
    srcs = ["ads_req_vin_390_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ads_shifter_115_test",
    size = "small",
    srcs = ["ads_shifter_115_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fail_241_test",
    size = "small",
    srcs = ["fail_241_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fbs1_243_test",
    size = "small",
    srcs = ["fbs1_243_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fbs2_240_test",
    size = "small",
    srcs = ["fbs2_240_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fbs3_237_test",
    size = "small",
    srcs = ["fbs3_237_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fbs4_235_test",
    size = "small",
    srcs = ["fbs4_235_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "status_310_test",
    size = "small",
    srcs = ["status_310_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vin_resp1_391_test",
    size = "small",
    srcs = ["vin_resp1_391_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vin_resp2_392_test",
    size = "small",
    srcs = ["vin_resp2_392_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vin_resp3_393_test",
    size = "small",
    srcs = ["vin_resp3_393_test.cc"],
    deps = [
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
