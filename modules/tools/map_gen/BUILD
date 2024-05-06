load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "add_signal",
    srcs = ["add_signal.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_overlap_py_pb2",
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/map_msgs:map_signal_py_pb2",
    ],
)

apollo_py_binary(
    name = "extract_path",
    srcs = ["extract_path.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "map_gen_single_lane",
    srcs = ["map_gen_single_lane.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_lane_py_pb2",
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/map_msgs:map_road_py_pb2",
    ],
)

apollo_py_binary(
    name = "map_gen_two_lanes_right_ext",
    srcs = ["map_gen_two_lanes_right_ext.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_lane_py_pb2",
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/map_msgs:map_road_py_pb2",
    ],
)

apollo_py_binary(
    name = "map_gen",
    srcs = ["map_gen.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_lane_py_pb2",
        "//modules/common_msgs/map_msgs:map_py_pb2",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/map_gen",
#     targets = [
#         "add_signal",
#         "extract_path",
#         "map_gen_single_lane",
#         "map_gen_two_lanes_right_ext",
#         "map_gen",
#     ]

# )

apollo_package()