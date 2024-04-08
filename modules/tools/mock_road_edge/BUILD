load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/proto:proto.bzl", "apollo_py_binary")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libmock_road_edge_component.so",
    srcs = [
        "mock_road_edge_component.cc",
    ],
    hdrs = [
        "mock_road_edge_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"mock_road_edge\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/map:apollo_map",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

filegroup(
    name = "mock_road_edge_conf",
    srcs = glob([
        "conf/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
    ]) + [":mock_road_edge_conf"],
)

apollo_package()

cpplint()