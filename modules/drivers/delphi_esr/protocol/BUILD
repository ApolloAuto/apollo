load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_delphi_esr_protocol",
    srcs = [
        "test_4e0.cc",
    ],
    hdrs = [
        "test_4e0.h",
    ],
    deps = [
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers:sensor_message_manager_base",
    ],
)

cc_test(
    name = "test_4e0_test",
    size = "small",
    srcs = [
        "test_4e0_test.cc",
    ],
    deps = [
        ":drivers_delphi_esr_protocol",
        "@gtest//:main",
    ],
)

cpplint()
