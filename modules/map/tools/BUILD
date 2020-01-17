load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "//cyber/common:file",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:macros",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_binary(
    name = "sim_map_generator",
    srcs = ["sim_map_generator.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:points_downsampler",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "proto_map_generator",
    srcs = ["proto_map_generator.cc"],
    deps = [
        "//cyber/common:file",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "bin_map_generator",
    srcs = ["bin_map_generator.cc"],
    deps = [
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "//modules/common/math",
        "//modules/common/util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
