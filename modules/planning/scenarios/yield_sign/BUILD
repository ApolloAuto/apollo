load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "yield_sign_scenario",
    srcs = [
        "stage_approach.cc",
        "stage_creep.cc",
        "yield_sign_scenario.cc",
    ],
    hdrs = [
        "stage_approach.h",
        "stage_creep.h",
        "yield_sign_scenario.h",
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
    name = "yield_sign_scenario_test",
    size = "small",
    srcs = ["yield_sign_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":yield_sign_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
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
        ":yield_sign_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
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
        ":yield_sign_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
    linkstatic = True,
)

cpplint()
