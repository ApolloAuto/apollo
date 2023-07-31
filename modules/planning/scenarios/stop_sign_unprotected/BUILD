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
#     data_dest = "planning-scenario-stop-sign-unprotected",
#     data = [
#         ":cyberfile.xml",
#         ":planning-scenario-stop-sign-unprotected.BUILD",
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
#     plugin = ":libstop_sign_unprotected_scenario.so",
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
#     dest = "planning-scenario-stop-sign-unprotected/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-scenario-stop-sign-unprotected/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_stop_sign_unprotected_scenario_description",
    description = ":plugins.xml",
    plugin = ":libstop_sign_unprotected_scenario.so",
)

cc_binary(
    name = "libstop_sign_unprotected_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":stop_sign_unprotected_scenario_lib"],
)

cc_library(
    name = "stop_sign_unprotected_scenario_lib",
    srcs = [
        "stage_creep.cc",
        "stage_intersection_cruise.cc",
        "stage_pre_stop.cc",
        "stage_stop.cc",
        "stop_sign_unprotected_scenario.cc",
    ],
    hdrs = [
        "stage_creep.h",
        "stage_intersection_cruise.h",
        "stage_pre_stop.h",
        "stage_stop.h",
        "stop_sign_unprotected_scenario.h",
        "context.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common/util",
        "//modules/planning/planning_base/common/util:util_lib",
        "//modules/planning/planning_base/scenario_base:scenario",
        "//modules/planning/scenarios/stop_sign_unprotected/proto:stop_sign_unprotected_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "stop_sign_unprotected_scenario_test",
    size = "small",
    srcs = ["stop_sign_unprotected_scenario_test.cc"],
    data = ["//modules/planning/planning_base:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_creep_test",
    size = "small",
    srcs = ["stage_creep_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_pre_stop_test",
    size = "small",
    srcs = ["stage_pre_stop_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_stop_test",
    size = "small",
    srcs = ["stage_stop_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":stop_sign_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
