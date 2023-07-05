load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib",
    targets = [
      ":libcanbus_common.so",
    ],
)

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
        "byte.cc",
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
        ":canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
