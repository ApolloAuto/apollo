load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_proto",
    ],
)

cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
    ],
)

cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_binary(
    name = "sim_map_generator",
    srcs = ["sim_map_generator.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:points_downsampler",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
    ],
)

cc_binary(
    name = "proto_map_generator",
    srcs = ["proto_map_generator.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
    ],
)

cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/math",
        "//modules/common/util",
    ],
)

cpplint()
