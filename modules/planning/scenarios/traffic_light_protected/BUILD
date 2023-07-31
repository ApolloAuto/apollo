load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

# install(
#     name = "install",
#     data_dest = "planning-scenario-traffic-light-protected",
#     data = [
#         ":cyberfile.xml",
#         ":planning-scenario-traffic-light-protected.BUILD",
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
#     plugin = ":libtraffic_light_protected_scenario.so",
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
#     dest = "planning-scenario-traffic-light-protected/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-scenario-traffic-light-protected/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_traffic_light_protected_scenario_description",
    description = ":plugins.xml",
    plugin = ":libtraffic_light_protected_scenario.so",
)

cc_binary(
    name = "libtraffic_light_protected_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":traffic_light_protected_scenario_lib"],
)

cc_library(
    name = "traffic_light_protected_scenario_lib",
    srcs = [
        "stage_approach.cc",
        "stage_intersection_cruise.cc",
        "traffic_light_protected_scenario.cc",
    ],
    hdrs = [
        "context.h",
        "stage_approach.h",
        "stage_intersection_cruise.h",
        "traffic_light_protected_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/scenario_base:scenario",
        "//modules/planning/planning_base/scenario_base/traffic_light_base:base_stage_traffic_light_cruise",
        "//modules/planning/scenarios/traffic_light_protected/proto:traffic_light_protected_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "traffic_light_protected_scenario_test",
    size = "small",
    srcs = ["traffic_light_protected_scenario_test.cc"],
    data = ["//modules/planning/planning_base:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":traffic_light_protected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_approach_test",
    size = "small",
    srcs = ["stage_approach_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":traffic_light_protected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
