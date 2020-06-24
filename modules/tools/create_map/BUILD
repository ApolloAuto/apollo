load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "convert_map_txt2bin",
    srcs = ["convert_map_txt2bin.py"],
    deps = [
        "//modules/map/proto:map_py_pb2",
    ],
)

py_binary(
    name = "create_map",
    srcs = ["create_map.py"],
    deps = [
        "//modules/map/proto:map_lane_py_pb2",
        "//modules/map/proto:map_py_pb2",
        "//modules/map/proto:map_road_py_pb2",
        "//modules/routing/proto:poi_py_pb2",
        "//modules/routing/proto:routing_py_pb2",
    ],
)

py_binary(
    name = "lane_recorder",
    srcs = ["lane_recorder.py"],
    deps = [
        "//modules/drivers/proto:mobileye_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/tools/common:logger",
    ],
)
