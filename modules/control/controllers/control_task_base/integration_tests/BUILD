load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_test_base",
    srcs = ["control_test_base.cc"],
    hdrs = ["control_test_base.h"],
    copts = ["-fno-access-control"],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/control/control_component:control_component_lib",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/controllers/control_task_base:control_task_agent",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "simple_control_test",
    size = "small",
    srcs = ["simple_control_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control/control_component:test_data"],
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
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

apollo_package()
cpplint()
