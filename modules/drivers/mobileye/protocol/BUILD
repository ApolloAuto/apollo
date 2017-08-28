load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_mobileye_protocol",
    srcs = [
        "details_738.cc",
    ],
    hdrs = [
        "details_738.h",
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

