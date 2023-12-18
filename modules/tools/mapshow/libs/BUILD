load("//tools/proto:proto.bzl", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "localization",
    srcs = ["localization.py"],
    deps = [
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_library(
    name = "map",
    srcs = ["map.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_library(
    name = "path",
    srcs = ["path.py"],
)

apollo_py_library(
    name = "planning",
    srcs = ["planning.py"],
    deps = [
        "//modules/common_msgs/planning_msgs:planning_internal_py_pb2",
    ],
)

apollo_py_library(
    name = "plot_planning",
    srcs = ["plot_planning.py"],
    deps = [
        ":localization",
        ":planning",
        ":subplot_path",
        ":subplot_sl_main",
        ":subplot_speed",
        ":subplot_st_main",
        ":subplot_st_speed",
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_library(
    name = "plot_smoothness",
    srcs = ["plot_smoothness.py"],
    deps = [
        ":planning",
        ":subplot_traj_acc",
        ":subplot_traj_path",
        ":subplot_traj_speed",
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_library(
    name = "plot_st",
    srcs = ["plot_st.py"],
    deps = [
        ":planning",
        ":subplot_st_main",
        ":subplot_st_speed",
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_library(
    name = "subplot_path",
    srcs = ["subplot_path.py"],
    deps = [
        ":map",
    ],
)

apollo_py_library(
    name = "subplot_sl_main",
    srcs = ["subplot_sl_main.py"],
)

apollo_py_library(
    name = "subplot_speed",
    srcs = ["subplot_speed.py"],
)

apollo_py_library(
    name = "subplot_st_main",
    srcs = ["subplot_st_main.py"],
)

apollo_py_library(
    name = "subplot_st_speed",
    srcs = ["subplot_st_speed.py"],
)

apollo_py_library(
    name = "subplot_traj_acc",
    srcs = ["subplot_traj_acc.py"],
)

apollo_py_library(
    name = "subplot_traj_path",
    srcs = ["subplot_traj_path.py"],
)

apollo_py_library(
    name = "subplot_traj_speed",
    srcs = ["subplot_traj_speed.py"],
)

apollo_package()