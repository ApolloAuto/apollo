load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control",
    srcs = ["sim_control.cc"],
    hdrs = [
        "sim_control.h",
        "sim_control_interface.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/proto:scenario_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "sim_control_test",
    size = "small",
    srcs = ["sim_control_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":sim_control",
        "//cyber/time:clock",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
