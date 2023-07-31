load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "traffic_rules",
    srcs = [
        "traffic_rule.cc",
    ],
    hdrs = [
        "traffic_rule.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/planning/planning_base/common:dependency_injector",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/common:config_util"
    ],
)

cc_library(
    name = "traffic_decider",
    srcs = ["traffic_decider.cc"],
    hdrs = ["traffic_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":traffic_rules",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/util:util_tool",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/reference_line",
        "//modules/planning/planning_base/traffic_rules_base/proto:traffic_rules_cc_proto",
        "//cyber/plugin_manager:plugin_manager",
        "//modules/planning/planning_base/common:config_util",
    ],
)

apollo_package()
cpplint()
