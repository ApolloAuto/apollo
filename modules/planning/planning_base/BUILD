load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "planning_testdata_install",
#     data_dest = if_gpu(
#         "planning-gpu/addition_data",
#         "planning/addition_data"
#     ),
#     data = [":planning_testdata"],
# )

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
        "//modules/planning/planning_base/common:history",
        "//modules/planning/planning_base/common:message_process",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/task_base/common:trajectory_optimizer"
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
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap",
        "//modules/planning/planning_base/common:dependency_injector",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:local_view",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory:publishable_trajectory",
        "//modules/planning/planning_base/planner:planner_dispatcher",
        "//modules/planning/planning_base/planner",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/traffic_rules_base:traffic_decider",
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
        "//modules/planning/planning_base/common:ego_info",
        "//modules/planning/planning_base/common:history",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:trajectory_stitcher",
        "//modules/planning/planning_base/common/util:util_lib",
        "//modules/planning/planning_base/planner:planner_dispatcher",
        "//modules/planning/planning_base/planner/navi:navi_planner",
        "//modules/planning/planning_base/planner/rtk:rtk_planner",
        "//modules/planning/planning_base/reference_line:reference_line_provider",
        "//modules/planning/planning_base/traffic_rules_base:traffic_decider",
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
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/planning_base/common:ego_info",
        "//modules/planning/planning_base/common:history",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:trajectory_stitcher",
        "//modules/planning/planning_base/common/smoothers:smoother",
        "//modules/planning/planning_base/common/util:util_lib",
        "//modules/planning/planning_base/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/planning/planning_base/planner:planner_dispatcher",
        "//modules/planning/planning_base/planner/rtk:rtk_planner",
        "//modules/planning/planning_base/proto:planning_semantic_map_config_cc_proto",
        "//modules/planning/planning_base/reference_line:reference_line_provider",
        "//modules/planning/planning_base/traffic_rules_base:traffic_decider",
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

# install(
#     name = "install",
#     data_dest = if_gpu(
#         "planning-gpu",
#         "planning"
#     ),
#     library_dest = if_gpu(
#         "planning-gpu/lib",
#         "planning/lib"
#     ),
#     data = [
#         ":runtime_data",
        
#     ] + if_gpu(
#         ["planning-gpu.BUILD", "cyberfile_gpu.xml",],
#         ["planning.BUILD", "cyberfile_cpu.xml",]
#     ),
#     rename= if_gpu(
#         {"planning-gpu/cyberfile_gpu.xml": "cyberfile.xml"}, 
#         {"planning/cyberfile_cpu.xml": "cyberfile.xml"}
#     ),
#     targets = [
#         ":libplanning_component.so",
#         "//modules/planning/planning_base/common:libplanning_gflags.so"
#     ],
#     deps = [
#         ":pb_planning",
#         ":planning_testdata_install",
#         "//modules/planning/planning_base/proto:py_pb_planning",
#         "//modules/planning/scenarios:install",
#         "//modules/planning/planning_base/common:install", 
#         "//modules/planning/tasks:install",
#         "//modules/planning/planning_base/proto:pb_hdrs_planning",
#     ],
# )

# install_files(
#     name = "pb_planning",
#     dest = if_gpu(
#         "planning-gpu",
#         "planning"
#     ),
#     files = [
#         "//modules/planning/planning_base/proto:auto_tuning_raw_feature_py_pb2",
#         "//modules/common_msgs/planning_msgs:decision_py_pb2",
#         "//modules/planning/planning_base/proto:ipopt_return_status_py_pb2",
#         "//modules/planning/planning_base/proto:lattice_structure_py_pb2",
#         "//modules/planning/planning_base/proto:learning_data_py_pb2",
#         "//modules/common_msgs/control_msgs:control_pad_msg_py_pb2",
#         "//modules/common_msgs/planning_msgs:planning_internal_py_pb2",
#         "//modules/common_msgs/planning_msgs:planning_py_pb2",
#         "//modules/planning/planning_base/proto:planning_semantic_map_config_py_pb2",
#         "//modules/planning/planning_base/proto:planning_status_py_pb2",
#         "//modules/planning/planning_base/proto:reference_line_smoother_config_py_pb2",
#         "//modules/common_msgs/planning_msgs:sl_boundary_py_pb2",
#         "//modules/planning/planning_base/proto:st_drivable_boundary_py_pb2",
#         "//modules/planning/planning_base/proto:traffic_rule_config_py_pb2",
#         "//modules/planning/planning_base/proto/math:cos_theta_smoother_config_py_pb2",
#         "//modules/planning/planning_base/proto/math:qp_problem_py_pb2",
#     ],
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_planning_src",
#         ":install_planning_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_planning_src",
#     src_dir = ["."],
#     dest = if_gpu(
#         "planning-gpu/src",
#         "planning/src"
#     ),
#     filter = "*",
# )

# install_src_files(
#     name = "install_planning_hdrs",
#     src_dir = ["."],
#     dest = if_gpu(
#         "planning-gpu/include",
#         "planning/include"
#     ),
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "planning/include",
#     data = [
#         "//modules/planning/planning_base/proto:st_drivable_boundary_py_pb2",
#         "//modules/planning/planning_base/proto:planning_semantic_map_config_py_pb2",
#         "//modules/planning/planning_base/proto:traffic_rule_config_py_pb2",
#         "//modules/planning/planning_base/proto:planning_status_py_pb2",
#         "//modules/planning/planning_base/proto:lattice_structure_py_pb2",
#         "//modules/planning/planning_base/proto:learning_data_py_pb2",
#         "//modules/planning/planning_base/proto:auto_tuning_model_input_py_pb2",
#         "//modules/planning/planning_base/proto:auto_tuning_raw_feature_py_pb2",
#         "//modules/planning/planning_base/proto:reference_line_smoother_config_py_pb2",
#         "//modules/planning/planning_base/proto:ipopt_return_status_py_pb2",
#         "//modules/planning/planning_base/proto/math:fem_pos_deviation_smoother_config_py_pb2",
#         "//modules/planning/planning_base/proto:planner_open_space_config_py_pb2",
#         "//modules/planning/planning_base/proto:navi_task_config_py_pb2",
#         "//modules/planning/planning_base/proto:planning_config_py_pb2",
#         "//modules/planning/planning_base/proto:plugin_declare_info_py_pb2",
#     ],
# )

apollo_package()
cpplint()
