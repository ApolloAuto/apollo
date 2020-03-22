load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_common",
    srcs = ["byte.cc"],
    hdrs = [
        "byte.h",
        "canbus_consts.h",
    ],
    deps = [
        "//modules/drivers/canbus:sensor_gflags",
    ],
)

cc_test(
    name = "byte_test",
    size = "small",
    srcs = ["byte_test.cc"],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
