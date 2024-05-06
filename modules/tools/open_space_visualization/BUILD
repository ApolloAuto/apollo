load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "distance_approach_python_interface",
    srcs = ["distance_approach_python_interface.py"],
)

apollo_py_binary(
    name = "distance_approach_visualizer",
    srcs = ["distance_approach_visualizer.py"],
    deps = [
        ":distance_approach_python_interface",
    ],
)

apollo_py_library(
    name = "hybrid_a_star_python_interface",
    srcs = ["hybrid_a_star_python_interface.py"],
)

apollo_py_binary(
    name = "hybrid_a_star_visualizer",
    srcs = ["hybrid_a_star_visualizer.py"],
    deps = [
        ":hybrid_a_star_python_interface",
    ],
)

apollo_py_library(
    name = "open_space_roi_interface",
    srcs = ["open_space_roi_interface.py"],
)

apollo_py_binary(
    name = "open_space_roi_visualizer",
    srcs = ["open_space_roi_visualizer.py"],
    deps = [
        ":open_space_roi_interface",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/open_space_visualization",
#     targets = [
#         ":open_space_roi_interface",
#         ":hybrid_a_star_python_interface",
#         ":distance_approach_python_interface",
#         ":distance_approach_visualizer",
#     ]
# )

apollo_package()