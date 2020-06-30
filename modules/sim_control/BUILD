load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control_component_lib",
    srcs = ["sim_control_component.cc"],
    hdrs = [
        "sim_control_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"sim_control\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/dreamview/backend/map:map_service",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/planning/proto:planning_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "libsim_control_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":sim_control_component_lib"],
)

cc_test(
    name = "sim_control_component_test",
    size = "small",
    srcs = ["sim_control_component_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":sim_control_component_lib",
        "//modules/common/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
