load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:common_util",
        "//modules/control/control_component/common:control_gflags",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/dreamview_msgs:chart_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/map_msgs:map_clear_area_cc_proto",
        "//modules/common_msgs/map_msgs:map_crosswalk_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/map_msgs:map_junction_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/map_msgs:map_overlap_cc_proto",
        "//modules/common_msgs/map_msgs:map_parking_space_cc_proto",
        "//modules/common_msgs/map_msgs:map_pnc_junction_cc_proto",
        "//modules/common_msgs/map_msgs:map_road_cc_proto",
        "//modules/common_msgs/map_msgs:map_signal_cc_proto",
        "//modules/common_msgs/map_msgs:map_speed_bump_cc_proto",
        "//modules/common_msgs/map_msgs:map_stop_sign_cc_proto",
        "//modules/common_msgs/map_msgs:map_yield_sign_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/common_msgs/planning_msgs:sl_boundary_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    linkstatic = True,
)

apollo_cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_component/common:control_gflags",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_proto",
    ],
    linkstatic = True,
)

apollo_cc_binary(
    name = "calibration_table_debug",
    srcs = ["calibration_table_debug.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_component/common:control_gflags",
        "//modules/common_msgs/control_msgs:control_cmd_proto",
        "//modules/common_msgs/chassis_msgs:chassis_proto",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/control/control_component/proto:calibration_debug_proto",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
