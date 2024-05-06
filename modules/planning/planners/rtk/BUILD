load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

apollo_plugin(
    name = "librtk_replay_planner.so",
    srcs = [
        "rtk_replay_planner.cc",
    ],
    hdrs = [
        "rtk_replay_planner.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_cc_test(
    name = "rtk_replay_planner_test",
    size = "small",
    srcs = ["rtk_replay_planner_test.cc"],
    data = [":testdata"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":rtk_replay_planner_lib",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
