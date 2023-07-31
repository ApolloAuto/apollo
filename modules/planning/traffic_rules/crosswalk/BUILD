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
#     data_dest = "planning-traffic-rules-crosswalk",
#     data = [
#         ":cyberfile.xml",
#         ":planning-traffic-rules-crosswalk.BUILD",
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
#     plugin = ":libcrosswalk.so",
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
#     dest = "planning-traffic-rules-crosswalk/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-traffic-rules-crosswalk/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_libcrosswalk_lib_description",
    description = ":plugins.xml",
    plugin = ":libcrosswalk.so",
)

cc_binary(
    name = "libcrosswalk.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":crosswalk_lib"],
)

cc_library(
    name = "crosswalk_lib",
    srcs = ["crosswalk.cc"],
    hdrs = ["crosswalk.h"],
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
        "//modules/planning/traffic_rules/crosswalk/proto:crosswalk_cc_proto"
    ],
)

apollo_package()
cpplint()
