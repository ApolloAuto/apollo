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
    name = "libpublic_road_planner.so",
    srcs = [
        "public_road_planner.cc",
        "scenario_manager.cc",
    ],
    hdrs = [
        "public_road_planner.h",
        "scenario_manager.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planners/public_road/proto:planner_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "public_road_planner_test",
    size = "small",
    srcs = ["public_road_planner_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":public_road_planner_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
