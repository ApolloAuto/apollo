load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "emergency_stop_scenario",
    srcs = [
        "emergency_stop_scenario.cc",
        "stage_approach.cc",
        "stage_standby.cc",
    ],
    hdrs = [
        "emergency_stop_scenario.h",
        "stage_approach.h",
        "stage_standby.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "emergency_stop_scenario_test",
    size = "small",
    srcs = ["emergency_stop_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_stop_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "stage_approach_test",
    size = "small",
    srcs = ["stage_approach_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_stop_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "stage_standby_test",
    size = "small",
    srcs = ["stage_standby_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_stop_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cpplint()
