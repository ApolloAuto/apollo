load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_perfect_control",
    srcs = ["sim_perfect_control.cc"],
    hdrs = [
        "sim_perfect_control.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

# cc_test(
#     name = "sim_control_test",
#     size = "small",
#     srcs = ["sim_control_test.cc"],
#     data = [
#         "//modules/dreamview/backend/testdata",
#     ],
#     deps = [
#         ":sim_perfect_control",
#         "//cyber",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
