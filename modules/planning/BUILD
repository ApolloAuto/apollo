load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":planning_component_lib"],
)

cc_library(
    name = "planning_component_lib",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":navi_planning",
        ":on_lane_planning",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:message_process",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
    ],
)

cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    copts = [
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util:future",
        "//modules/common/util:message_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
    ],
)

cc_library(
    name = "navi_planning",
    srcs = ["navi_planning.cc"],
    hdrs = ["navi_planning.h"],
    copts = [
        "-fopenmp",
    ],
    deps = [
        ":planning_base",
    ],
)

cc_library(
    name = "on_lane_planning",
    srcs = ["on_lane_planning.cc"],
    hdrs = ["on_lane_planning.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "//cyber/common:log",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
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

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libplanning_component.so",
    ],
    deps = [
        ":pb_planning",
        "//cyber:install",
    ],
)

install_files(
    name = "pb_planning",
    dest = "modules/planning",
    files = [
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/planning/proto:decision_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/planning/proto:open_space_task_config_py_pb2",
        "//modules/planning/proto:pad_msg_py_pb2",
        "//modules/planning/proto:planner_open_space_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
        "//modules/planning/proto:planning_internal_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:sl_boundary_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:task_config_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
    ],
)

cpplint()
