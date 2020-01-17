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
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/planning/proto:planning_proto",
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
        "//modules/common/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
