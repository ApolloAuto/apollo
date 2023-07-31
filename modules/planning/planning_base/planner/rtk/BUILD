load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_planner",
    srcs = ["rtk_replay_planner.cc"],
    hdrs = ["rtk_replay_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planning_base/planner",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

cc_test(
    name = "rtk_replay_planner_test",
    size = "small",
    srcs = ["rtk_replay_planner_test.cc"],
    data = ["//modules/planning/planning_base:planning_testdata"],
    linkopts = ["-lgomp"],
    deps = [
        ":rtk_planner",
        "//cyber",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
        "//modules/planning/planning_base/common:dependency_injector",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
