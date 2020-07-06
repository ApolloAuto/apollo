load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "planning_proto",
    deps = [
        ":planning_proto_lib",
    ],
)

proto_library(
    name = "planning_proto_lib",
    srcs = [
        "pad_msg.proto",
        "planning.proto",
    ],
    deps = [
        ":decision_proto_lib",
        ":planning_internal_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "task_config_proto",
    deps = [
        ":task_config_proto_lib",
    ],
)

proto_library(
    name = "task_config_proto_lib",
    srcs = ["task_config.proto"],
)

cc_proto_library(
    name = "open_space_task_config_proto",
    deps = [
        ":open_space_task_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_task_config_proto_lib",
    srcs = ["open_space_task_config.proto"],
    deps = [
        ":planner_open_space_config_proto_lib",
    ],
)

cc_proto_library(
    name = "decision_proto",
    deps = [
        ":decision_proto_lib",
    ],
)

proto_library(
    name = "decision_proto_lib",
    srcs = ["decision.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "planning_config_proto",
    deps = [
        ":planning_config_proto_lib",
    ],
)

proto_library(
    name = "planning_config_proto_lib",
    srcs = ["planning_config.proto"],
    deps = [
        ":open_space_task_config_proto_lib",
        ":task_config_proto_lib",
    ],
)

proto_library(
    name = "planning_internal_proto_lib",
    srcs = ["planning_internal.proto"],
    deps = [
        ":decision_proto_lib",
        ":planning_config_proto_lib",
        ":sl_boundary_proto_lib",
        ":st_drivable_boundary_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/dreamview/proto:chart_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/map/relative_map/proto:navigation_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "planning_stats_proto",
    deps = [
        ":planning_stats_proto_lib",
    ],
)

proto_library(
    name = "planning_stats_proto_lib",
    srcs = ["planning_stats.proto"],
)

cc_proto_library(
    name = "reference_line_smoother_config_proto",
    deps = [
        ":reference_line_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "reference_line_smoother_config_proto_lib",
    srcs = ["reference_line_smoother_config.proto"],
    deps = [
        "//modules/planning/proto/math:cos_theta_smoother_config_proto_lib",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "sl_boundary_proto_lib",
    srcs = ["sl_boundary.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

proto_library(
    name = "st_drivable_boundary_proto_lib",
    srcs = ["st_drivable_boundary.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "traffic_rule_config_proto",
    deps = [
        ":traffic_rule_config_proto_lib",
    ],
)

proto_library(
    name = "traffic_rule_config_proto_lib",
    srcs = ["traffic_rule_config.proto"],
)

cc_proto_library(
    name = "planning_status_proto",
    deps = [
        ":planning_status_proto_lib",
    ],
)

proto_library(
    name = "planning_status_proto_lib",
    srcs = ["planning_status.proto"],
    deps = [
        ":planning_config_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "lattice_structure_proto",
    deps = [
        ":lattice_structure_proto_lib",
    ],
)

proto_library(
    name = "lattice_structure_proto_lib",
    srcs = ["lattice_structure.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

proto_library(
    name = "planner_open_space_config_proto_lib",
    srcs = ["planner_open_space_config.proto"],
    deps = [
        ":task_config_proto_lib",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_proto_lib",
    ],
)

cc_proto_library(
    name = "planner_open_space_config_proto",
    deps = [
        ":planner_open_space_config_proto_lib",
    ],
)

cc_proto_library(
    name = "auto_tuning_raw_feature_proto",
    deps = [
        ":auto_tuning_raw_feature_lib",
    ],
)

proto_library(
    name = "auto_tuning_raw_feature_lib",
    srcs = ["auto_tuning_raw_feature.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "auto_tuning_model_input_proto",
    deps = [
        ":auto_tuning_model_input_lib",
    ],
)

proto_library(
    name = "auto_tuning_model_input_lib",
    srcs = ["auto_tuning_model_input.proto"],
)

cc_proto_library(
    name = "ipopt_return_status_proto",
    deps = [
        ":ipopt_return_status_proto_lib",
    ],
)

proto_library(
    name = "ipopt_return_status_proto_lib",
    srcs = ["ipopt_return_status.proto"],
)

cc_proto_library(
    name = "learning_data_proto",
    deps = [
        ":learning_data_proto_lib",
    ],
)

proto_library(
    name = "learning_data_proto_lib",
    srcs = ["learning_data.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:feature_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "planning_semantic_map_config_proto",
    deps = [
        ":planning_semantic_map_config_proto_lib",
    ],
)

proto_library(
    name = "planning_semantic_map_config_proto_lib",
    srcs = ["planning_semantic_map_config.proto"],
)

py_proto(
    name = "pad_msg_pb2",
    src = "pad_msg.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "lattice_structure_pb2",
    src = "lattice_structure.proto",
)

py_proto(
    name = "planning_stats_pb2",
    src = "planning_stats.proto",
)

py_proto(
    name = "decision_pb2",
    src = "decision.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:vehicle_signal_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "open_space_task_config_pb2",
    src = "open_space_task_config.proto",
)

py_proto(
    name = "task_config_pb2",
    src = "task_config.proto",
)

py_proto(
    name = "st_drivable_boundary_pb2",
    src = "st_drivable_boundary.proto",
)

py_proto(
    name = "planning_pb2",
    src = "planning.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/map/proto:map_id_pb2",
        "//modules/planning/proto:decision_pb2",
        "//modules/planning/proto:planning_internal_pb2",
    ],
)

py_proto(
    name = "sl_boundary_pb2",
    src = "sl_boundary.proto",
    deps = [
        "//modules/common/proto:pnc_point_pb2",
    ],
)

py_proto(
    name = "reference_line_smoother_config_pb2",
    src = "reference_line_smoother_config.proto",
    deps = [
        "//modules/planning/proto/math:cos_theta_smoother_config_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_pb2",
    ],
)

py_proto(
    name = "planning_status_pb2",
    src = "planning_status.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/planning/proto:planning_config_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "ipopt_return_status_pb2",
    src = "ipopt_return_status.proto",
)

py_proto(
    name = "traffic_rule_config_pb2",
    src = "traffic_rule_config.proto",
)

py_proto(
    name = "auto_tuning_model_input_pb2",
    src = "auto_tuning_model_input.proto",
)

py_proto(
    name = "planning_internal_pb2",
    src = "planning_internal.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/dreamview/proto:chart_pb2",
        "//modules/localization/proto:localization_pb2",
        "//modules/map/relative_map/proto:navigation_pb2",
        "//modules/perception/proto:traffic_light_detection_pb2",
        "//modules/planning/proto:decision_pb2",
        "//modules/planning/proto:planning_config_pb2",
        "//modules/planning/proto:sl_boundary_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "learning_data_pb2",
    src = "learning_data.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/perception/proto:traffic_light_detection_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "planner_open_space_config_pb2",
    src = "planner_open_space_config.proto",
    deps = [
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_pb2",
        "//modules/planning/proto:task_config_pb2",
    ],
)

py_proto(
    name = "planning_config_pb2",
    src = "planning_config.proto",
    deps = [
        "//modules/planning/proto:open_space_task_config_pb2",
        "//modules/planning/proto:task_config_pb2",
    ],
)

py_proto(
    name = "auto_tuning_raw_feature_pb2",
    src = "auto_tuning_raw_feature.proto",
    deps = [
        "//modules/common/proto:pnc_point_pb2",
    ],
)

py_proto(
    name = "planning_semantic_map_config_pb2",
    src = "planning_semantic_map_config.proto",
)
