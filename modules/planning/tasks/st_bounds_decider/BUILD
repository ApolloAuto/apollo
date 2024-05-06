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
#     data_dest = "planning-task-st-bounds-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-st-bounds-decider.BUILD",
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
#     plugin = ":libst_bounds_decider.so",
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
#     dest = "planning-task-st-bounds-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-st-bounds-decider/include",
#     filter = "*.h",
# )

apollo_plugin(
    name = "libst_bounds_decider.so",
    srcs = [
        "st_bounds_decider.cc",
        "st_driving_limits.cc",
        "st_guide_line.cc",
        "st_obstacles_processor.cc",
    ],
    hdrs = [
        "st_bounds_decider.h",
        "st_driving_limits.h",
        "st_guide_line.h",
        "st_obstacles_processor.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/tasks/st_bounds_decider/proto:st_bounds_decider_cc_proto",
    ],
)

apollo_package()

cpplint()
