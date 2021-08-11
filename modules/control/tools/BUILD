load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/dreamview/proto:chart_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/map/proto:map_cc_proto",
        "//modules/map/proto:map_clear_area_cc_proto",
        "//modules/map/proto:map_crosswalk_cc_proto",
        "//modules/map/proto:map_geometry_cc_proto",
        "//modules/map/proto:map_id_cc_proto",
        "//modules/map/proto:map_junction_cc_proto",
        "//modules/map/proto:map_lane_cc_proto",
        "//modules/map/proto:map_overlap_cc_proto",
        "//modules/map/proto:map_parking_space_cc_proto",
        "//modules/map/proto:map_pnc_junction_cc_proto",
        "//modules/map/proto:map_road_cc_proto",
        "//modules/map/proto:map_signal_cc_proto",
        "//modules/map/proto:map_speed_bump_cc_proto",
        "//modules/map/proto:map_stop_sign_cc_proto",
        "//modules/map/proto:map_yield_sign_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/proto:decision_cc_proto",
        "//modules/planning/proto:open_space_task_config_cc_proto",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_internal_cc_proto",
        "//modules/planning/proto:sl_boundary_cc_proto",
        "//modules/planning/proto:task_config_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/control/common",
        "//modules/control/proto:pad_msg_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
