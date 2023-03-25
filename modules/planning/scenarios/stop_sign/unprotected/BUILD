load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_sign_unprotected_scenario",
    srcs = [
        "stage_creep.cc",
        "stage_intersection_cruise.cc",
        "stage_pre_stop.cc",
        "stage_stop.cc",
        "stop_sign_unprotected_scenario.cc",
    ],
    hdrs = [
        "stage_creep.h",
        "stage_intersection_cruise.h",
        "stage_pre_stop.h",
        "stage_stop.h",
        "stop_sign_unprotected_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common/util",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/common:stage_intersection_cruise_impl",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "//modules/planning/tasks/deciders/creep_decider",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "stop_sign_unprotected_scenario_test",
    size = "small",
    srcs = ["stop_sign_unprotected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "stage_creep_test",
    size = "small",
    srcs = ["stage_creep_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "stage_pre_stop_test",
    size = "small",
    srcs = ["stage_pre_stop_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "stage_stop_test",
    size = "small",
    srcs = ["stage_stop_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cpplint()
