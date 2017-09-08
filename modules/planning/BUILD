load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_impl",
    srcs = [
        "planning.cc",
        "planning_node.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/proto:path_point_proto",
        "//modules/common/util:factory",
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
        ":planning_impl",
        "//external:gflags",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
