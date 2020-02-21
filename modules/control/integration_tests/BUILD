load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_test_base",
    srcs = ["control_test_base.cc"],
    hdrs = ["control_test_base.h"],
    deps = [
        "//cyber",
        "//modules/common/proto:geometry_proto",
        "//modules/common/time",
        "//modules/control:control_component_lib",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "simple_control_test",
    size = "small",
    srcs = ["simple_control_test.cc"],
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
    data = ["//modules/control:control_testdata"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
