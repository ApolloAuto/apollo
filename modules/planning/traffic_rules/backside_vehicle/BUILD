load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)
# install(
#     name = "install",
#     data_dest = "planning-traffic-rules-backside-vehicle",
#     data = [
#         ":cyberfile.xml",
#         ":planning-traffic-rules-backside-vehicle.BUILD",
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
#     plugin = ":libbackside_vehicle.so",
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
#     dest = "planning-traffic-rules-backside-vehicle/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-traffic-rules-backside-vehicle/include",
#     filter = "*.h",
# )
cyber_plugin_description(
    name = "plugin_libbackside_vehicle_description",
    description = ":plugins.xml",
    plugin = ":libbackside_vehicle.so",
)

cc_binary(
    name = "libbackside_vehicle.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":backside_vehicle_lib"],
)

cc_library(
    name = "backside_vehicle_lib",
    srcs = ["backside_vehicle.cc"],
    hdrs = ["backside_vehicle.h"],
    alwayslink = True,
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/planning/planning_base/common:dependency_injector",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/traffic_rules_base:traffic_rules",
        "//modules/planning/planning_base/common:config_util",
        "//modules/planning/traffic_rules/backside_vehicle/proto:backside_vehicle_cc_proto"
    ],
)

apollo_package()
cpplint()
