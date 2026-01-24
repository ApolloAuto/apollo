load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libsquare_scenario.so",
    srcs = [
        "extricate_stage.cc",
        "square_lane_follow_stage.cc",
        "square_scenario.cc",
    ],
    hdrs = [
      "extricate_stage.h",
        "square_lane_follow_stage.h",
        "square_scenario.h",
        "context.h"
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/square/proto:square_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package(enable_source=False)

cpplint()
