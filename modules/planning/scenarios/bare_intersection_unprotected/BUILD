load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

# install(
#     name = "install",
#     data_dest = "planning-scenario-bare-intersection-unprotected",
#     data = [
#         ":cyberfile.xml",
#         ":planning-scenario-bare-intersection-unprotected.BUILD",
#     ],
#     deps = [
#         "install_plugin",
#     ],
# )

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

# install_plugin(
#     name = "install_plugin",
#     data = [
#     ] + glob(["conf/**"]),
#     description = ":plugins.xml",
#     plugin = ":libbare_intersection_unprotected_scenario.so",
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
#     dest = "planning-scenario-bare-intersection-unprotected/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-scenario-bare-intersection-unprotected/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_bare_intersection_unprotected_scenario_description",
    description = ":plugins.xml",
    plugin = ":libbare_intersection_unprotected_scenario.so",
)

cc_binary(
    name = "libbare_intersection_unprotected_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = PLANNING_COPTS,
    deps = [":bare_intersection_unprotected_scenario_lib"],
)

cc_library(
    name = "bare_intersection_unprotected_scenario_lib",
    srcs = [
        "bare_intersection_unprotected_scenario.cc",
        "stage_approach.cc",
        "stage_intersection_cruise.cc",
    ],
    hdrs = [
        "bare_intersection_unprotected_scenario.h",
        "stage_approach.h",
        "stage_intersection_cruise.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/scenario_base:scenario",
        "//modules/planning/scenarios/bare_intersection_unprotected/proto:bare_intersection_unprotected_scenario_cc_proto",
        "//modules/planning/planning_base/scenario_base:stage",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "bare_intersection_unprotected_scenario_test",
    size = "small",
    srcs = ["bare_intersection_unprotected_scenario_test.cc"],
    data = ["//modules/planning/planning_base:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":bare_intersection_unprotected_scenario_lib",
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
        ":bare_intersection_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
