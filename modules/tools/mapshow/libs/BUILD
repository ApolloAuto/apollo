load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "localization",
    srcs = ["localization.py"],
    deps = [
        "//modules/localization/proto:localization_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "map",
    srcs = ["map.py"],
    deps = [
        "//modules/map/proto:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "path",
    srcs = ["path.py"],
)

py_library(
    name = "planning",
    srcs = ["planning.py"],
    deps = [
        "//modules/planning/proto:planning_internal_py_pb2",
    ],
)

py_library(
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
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_library(
    name = "plot_smoothness",
    srcs = ["plot_smoothness.py"],
    deps = [
        ":planning",
        ":subplot_traj_acc",
        ":subplot_traj_path",
        ":subplot_traj_speed",
        "//cyber/python/cyber_py3:cyber",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_library(
    name = "plot_st",
    srcs = ["plot_st.py"],
    deps = [
        ":planning",
        ":subplot_st_main",
        ":subplot_st_speed",
        "//cyber/python/cyber_py3:cyber",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_library(
    name = "subplot_path",
    srcs = ["subplot_path.py"],
    deps = [
        ":map",
    ],
)

py_library(
    name = "subplot_sl_main",
    srcs = ["subplot_sl_main.py"],
)

py_library(
    name = "subplot_speed",
    srcs = ["subplot_speed.py"],
)

py_library(
    name = "subplot_st_main",
    srcs = ["subplot_st_main.py"],
)

py_library(
    name = "subplot_st_speed",
    srcs = ["subplot_st_speed.py"],
)

py_library(
    name = "subplot_traj_acc",
    srcs = ["subplot_traj_acc.py"],
)

py_library(
    name = "subplot_traj_path",
    srcs = ["subplot_traj_path.py"],
)

py_library(
    name = "subplot_traj_speed",
    srcs = ["subplot_traj_speed.py"],
)
