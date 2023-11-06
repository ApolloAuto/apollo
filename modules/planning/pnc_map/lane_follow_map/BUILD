load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "liblane_follow_map.so",
    srcs = ["lane_follow_map.cc",],
    hdrs = ["lane_follow_map.h",],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/map:apollo_map",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
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

apollo_package()
cpplint()
