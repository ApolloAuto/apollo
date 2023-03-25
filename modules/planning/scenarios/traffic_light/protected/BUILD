load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_protected_scenario",
    srcs = [
        "stage_approach.cc",
        "stage_intersection_cruise.cc",
        "traffic_light_protected_scenario.cc",
    ],
    hdrs = [
        "stage_approach.h",
        "stage_intersection_cruise.h",
        "traffic_light_protected_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/common:stage_intersection_cruise_impl",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_test(
    name = "traffic_light_protected_scenario_test",
    size = "small",
    srcs = ["traffic_light_protected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":traffic_light_protected_scenario",
        "@com_google_googletest//:gtest_main",
    ],
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
        ":traffic_light_protected_scenario",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
