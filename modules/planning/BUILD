load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_planning",
    srcs = [
        "planning.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common:apollo_app",
        "//modules/common/proto:path_point_proto",
        "//modules/common/vehicle_state",
        "//modules/decision/proto:decision_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:lib_planning_common",
        "//modules/planning/planner:lib_planner",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "planning",
    srcs = ["main.cc"],
    deps = [
        ":lib_planning",
        "//external:gflags",
    ],
)

cc_test(
    name = "planning_test",
    size = "small",
    srcs = [
        "planning_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":lib_planning",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
