load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)
# install(
#     name = "install",
#     data_dest = "planning-task-path-time-heuristic",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-path-time-heuristic.BUILD",
#     ],
#     deps = [
#         "install_plugin",
#     ],
# )

# install_plugin(
#     name = "install_plugin",
#     data = [
#     ] + glob(["conf/**"]),
#     description = ":plugins.xml",
#     plugin = ":libpath_time_heuristic_optimizer.so",
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_all_src",
#         ":install_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_all_src",
#     src_dir = ["."],
#     dest = "planning-task-path-time-heuristic/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-path-time-heuristic/include",
#     filter = "*.h",
# )

apollo_cc_test(
    name = "gridded_path_time_graph_test",
    size = "small",
    srcs = ["gridded_path_time_graph_test.cc"],
    
    deps = [
        ":path_time_heuristic_optimizer_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_plugin(
    name = "libpath_time_heuristic_optimizer.so",
    srcs = [
        "dp_st_cost.cc",
        "gridded_path_time_graph.cc",
        "path_time_heuristic_optimizer.cc",
        "st_graph_point.cc",
    ],
    hdrs = [
        "dp_st_cost.h",
        "gridded_path_time_graph.h",
        "path_time_heuristic_optimizer.h",
        "st_graph_point.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/tasks/path_time_heuristic/proto:path_time_heuristic_cc_proto",
        "//modules/planning/tasks/speed_bounds_decider:st_boundary_mapper",
    ],
)

apollo_package()

cpplint()