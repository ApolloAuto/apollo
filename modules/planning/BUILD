load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "planning_testdata_install",
    data_dest = if_gpu(
        "planning-gpu/addition_data",
        "planning/addition_data"
    ),
    data = [":planning_testdata"],
)

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = True,
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
        "//modules/common/util:util_tool",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:message_process",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
    ],
    alwayslink = True,
)


cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\""
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/dreamview_msgs:chart_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:local_view",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task_factory",
    ],
)

cc_library(
    name = "navi_planning",
    srcs = ["navi_planning.cc"],
    hdrs = ["navi_planning.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\""
    ],
    deps = [
        ":planning_base",
        "//cyber",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/traffic_rules:traffic_decider",
        "@com_google_protobuf//:protobuf",
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
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/proto:planning_semantic_map_config_cc_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
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
    data_dest = if_gpu(
        "planning-gpu",
        "planning"
    ),
    library_dest = if_gpu(
        "planning-gpu/lib",
        "planning/lib"
    ),
    data = [
        ":runtime_data",
        
    ] + if_gpu(
        ["planning-gpu.BUILD", "cyberfile_gpu.xml",],
        ["planning.BUILD", "cyberfile_cpu.xml",]
    ),
    rename= if_gpu(
        {"planning-gpu/cyberfile_gpu.xml": "cyberfile.xml"}, 
        {"planning/cyberfile_cpu.xml": "cyberfile.xml"}
    ),
    targets = [
        ":libplanning_component.so",
    ],
    deps = [
        ":pb_planning",
        ":planning_testdata_install",
        "//modules/planning/proto:py_pb_planning",
        "//modules/planning/proto:pb_hdrs_planning",
    ],
)

install_files(
    name = "pb_planning",
    dest = if_gpu(
        "planning-gpu",
        "planning"
    ),
    files = [
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/common_msgs/planning_msgs:decision_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/common_msgs/control_msgs:pad_msg_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_internal_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/common_msgs/planning_msgs:sl_boundary_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_planning_src",
        ":install_planning_hdrs"
    ],
)

install_src_files(
    name = "install_planning_src",
    src_dir = ["."],
    dest = if_gpu(
        "planning-gpu/src",
        "planning/src"
    ),
    filter = "*",
)

install_src_files(
    name = "install_planning_hdrs",
    src_dir = ["."],
    dest = if_gpu(
        "planning-gpu/include",
        "planning/include"
    ),
    filter = "*.h",
)

install(
    name = "pb_hdrs",
    data_dest = "planning/include",
    data = [
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/planning/proto:auto_tuning_model_input_py_pb2",
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_py_pb2",
        "//modules/planning/proto:open_space_task_config_py_pb2",
        "//modules/planning/proto:planner_open_space_config_py_pb2",
        "//modules/planning/proto:task_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
    ],
)

cpplint()
