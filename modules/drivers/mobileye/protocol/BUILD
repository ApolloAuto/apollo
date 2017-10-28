load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_mobileye_protocol",
    srcs = [
        "aftermarket_669.cc",
        "details_737.cc",
        "details_738.cc",
        "details_739.cc",
        "details_73a.cc",
        "details_73b.cc",
        "lka_766.cc",
        "lka_767.cc",
        "lka_768.cc",
        "lka_769.cc",
        "next_76c.cc",
        "next_76d.cc",
        "num_76b.cc",
        "reference_76a.cc",
    ],
    hdrs = [
        "aftermarket_669.h",
        "details_737.h",
        "details_738.h",
        "details_739.h",
        "details_73a.h",
        "details_73b.h",
        "lka_766.h",
        "lka_767.h",
        "lka_768.h",
        "lka_769.h",
        "next_76c.h",
        "next_76d.h",
        "num_76b.h",
        "reference_76a.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_test(
    name = "aftermarket_669_test",
    size = "small",
    srcs = [
        "aftermarket_669_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "details_737_test",
    size = "small",
    srcs = [
        "details_737_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "details_738_test",
    size = "small",
    srcs = [
        "details_738_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "details_739_test",
    size = "small",
    srcs = [
        "details_739_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "details_73a_test",
    size = "small",
    srcs = [
        "details_73a_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "details_73b_test",
    size = "small",
    srcs = [
        "details_73b_test.cc",
    ],
    deps = [
        ":drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cpplint()
