load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
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
#     data_dest = "planning-scenario-park-and-go",
#     data = [
#         ":cyberfile.xml",
#         ":planning-scenario-park-and-go.BUILD",
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
#     plugin = ":libpark_and_go_scenario.so",
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
#     dest = "planning-scenario-park-and-go/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-scenario-park-and-go/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_park_and_go_description",
    description = ":plugins.xml",
    plugin = ":libpark_and_go_scenario.so",
)

cc_binary(
    name = "libpark_and_go_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":park_and_go_scenario_lib"],
)

cc_library(
    name = "park_and_go_scenario_lib",
    srcs = [
        "park_and_go_scenario.cc",
        "stage_adjust.cc",
        "stage_check.cc",
        "stage_cruise.cc",
        "stage_pre_cruise.cc",
        "util.cc",
    ],
    hdrs = [
        "context.h",
        "park_and_go_scenario.h",
        "stage_adjust.h",
        "stage_check.h",
        "stage_cruise.h",
        "stage_pre_cruise.h",
        "util.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/common/util:util_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/scenario_base:scenario",
        "//modules/planning/scenarios/park_and_go/proto:park_and_go_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

cc_test(
    name = "park_and_go_scenario_test",
    size = "small",
    srcs = ["park_and_go_scenario_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":park_and_go_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "park_and_go_stages_test",
    size = "small",
    srcs = [
        "stage_adjust_test.cc",
        "stage_check_test.cc",
        "stage_cruise_test.cc",
        "stage_pre_cruise_test.cc",
    ],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":park_and_go_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
