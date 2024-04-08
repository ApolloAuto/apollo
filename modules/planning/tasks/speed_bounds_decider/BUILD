load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)
# install(
#     name = "install",
#     data_dest = "planning-task-speed-bounds-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-speed-bounds-decider.BUILD",
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
#     plugin = ":libspeed_bounds_decider.so",
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
#     dest = "lanning-task-speed-bounds-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "lanning-task-speed-bounds-decider/include",
#     filter = "*.h",
# )

apollo_cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
)

apollo_cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = ["st_boundary_mapper_test.cc"],
    deps = [
        ":st_boundary_mapper",
        "//cyber",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_plugin(
    name = "libspeed_bounds_decider.so",
    srcs = ["speed_bounds_decider.cc"],
    hdrs = ["speed_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        ":st_boundary_mapper",
        "//modules/common/status",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
)

apollo_package()

cpplint()
