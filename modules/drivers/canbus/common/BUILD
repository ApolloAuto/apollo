load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcanbus_common.so",
    srcs = [
        "byte.cc",
        "byte.h",
        "canbus_consts.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//modules/drivers/canbus:sensor_gflags",
    ],
)

cc_library(
    name = "canbus_common",
    srcs = ["libcanbus_common.so"],
    hdrs = [
        "byte.h",
        "canbus_consts.h",
    ],
)

cc_test(
    name = "byte_test",
    size = "small",
    srcs = ["byte_test.cc"],
    deps = [
        ":canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
