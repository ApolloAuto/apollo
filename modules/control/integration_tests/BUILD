load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_test_base",
    srcs = ["control_test_base.cc"],
    hdrs = ["control_test_base.h"],
    copts = ["-fno-access-control"],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/control:control_component_lib",
        "//modules/control/common:control_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "simple_control_test",
    size = "small",
    srcs = ["simple_control_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "relative_position_test",
    size = "small",
    srcs = ["relative_position_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = ["mpc_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
