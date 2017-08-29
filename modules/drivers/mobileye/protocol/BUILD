load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_mobileye_protocol",
    srcs = [
        "details_738.cc",
        "details_739.cc",
        "details_73a.cc",
        "details_73b.cc",
    ],
    hdrs = [
        "details_738.h",
        "details_739.h",
        "details_73a.h",
        "details_73b.h",
    ],
    deps = [
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers:sensor_message_manager_base",
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
