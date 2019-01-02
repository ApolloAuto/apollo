load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_component_lib",
    srcs = [
        "planning_component.cc",
    ],
    hdrs = [
        "planning_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":planning_component_lib"],
)

cc_library(
    name = "planning_lib",
    srcs = [
        "navi_planning.cc",
        "open_space_planning.cc",
        "planning_base.cc",
        "std_planning.cc",
    ],
    hdrs = [
        "navi_planning.h",
        "open_space_planning.h",
        "planning_base.h",
        "std_planning.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:message_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:trajectory_stitcher",
        "//modules/planning/planner",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:traffic_rule_config_proto",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/tasks/smoothers:smoother",
        "//modules/planning/traffic_rules:traffic_decider",
        "//modules/prediction/proto:prediction_proto",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()
