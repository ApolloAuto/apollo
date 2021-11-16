load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "distance_approach_python_interface",
    srcs = ["distance_approach_python_interface.py"],
    data = [
        "//modules/planning/open_space/tools:distance_approach_problem_wrapper_lib.so",
    ],
)

py_binary(
    name = "distance_approach_visualizer",
    srcs = ["distance_approach_visualizer.py"],
    deps = [
        ":distance_approach_python_interface",
    ],
)

py_library(
    name = "hybrid_a_star_python_interface",
    srcs = ["hybrid_a_star_python_interface.py"],
    data = [
        "//modules/planning/open_space/tools:hybrid_a_star_wrapper_lib.so",
    ],
)

py_binary(
    name = "hybrid_a_star_visualizer",
    srcs = ["hybrid_a_star_visualizer.py"],
    deps = [
        ":hybrid_a_star_python_interface",
    ],
)

py_library(
    name = "open_space_roi_interface",
    srcs = ["open_space_roi_interface.py"],
    data = [
        "//modules/planning/open_space/tools:open_space_roi_wrapper_lib.so",
    ],
)

py_binary(
    name = "open_space_roi_visualizer",
    srcs = ["open_space_roi_visualizer.py"],
    deps = [
        ":open_space_roi_interface",
    ],
)
