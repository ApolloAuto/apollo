load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/map:apollo_map",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util:common_util",
        "//modules/map:apollo_map",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "//cyber",
        "//modules/map:apollo_map",
        "//modules/common_msgs/routing_msgs:poi_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
    ],
)

apollo_cc_binary(
    name = "sim_map_generator",
    srcs = ["sim_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/util:common_util",
        "//modules/map:apollo_map",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
    ],
)

apollo_cc_binary(
    name = "proto_map_generator",
    srcs = ["proto_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/map:apollo_map",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "bin_map_generator",
    srcs = ["bin_map_generator.cc"],
    deps = [
        "//cyber",
        "//modules/map:apollo_map",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "//modules/common/math",
        "//modules/common/util:common_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
