load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "add_signal",
    srcs = ["add_signal.py"],
    deps = [
        "//modules/map/proto:map_overlap_py_pb2",
        "//modules/map/proto:map_py_pb2",
        "//modules/map/proto:map_signal_py_pb2",
    ],
)

py_binary(
    name = "extract_path",
    srcs = ["extract_path.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "map_gen_single_lane",
    srcs = ["map_gen_single_lane.py"],
    deps = [
        "//modules/map/proto:map_lane_py_pb2",
        "//modules/map/proto:map_py_pb2",
        "//modules/map/proto:map_road_py_pb2",
    ],
)

py_binary(
    name = "map_gen_two_lanes_right_ext",
    srcs = ["map_gen_two_lanes_right_ext.py"],
    deps = [
        "//modules/map/proto:map_lane_py_pb2",
        "//modules/map/proto:map_py_pb2",
        "//modules/map/proto:map_road_py_pb2",
    ],
)

py_binary(
    name = "map_gen",
    srcs = ["map_gen.py"],
    deps = [
        "//modules/map/proto:map_lane_py_pb2",
        "//modules/map/proto:map_py_pb2",
    ],
)
