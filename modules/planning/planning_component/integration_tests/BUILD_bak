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
    name = "libplanning_test_base.so",
    srcs = [
        "planning_test_base.cc",
    ],
    hdrs = [
        "planning_test_base.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/planning/planning_component:planning_component_lib",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planners/public_road/proto:planner_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "garage_test",
    size = "small",
    srcs = ["garage_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":planning_test_base_lib",
        "//modules/planning/planning_component:planning_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "sunnyvale_loop_test",
    size = "small",
    srcs = ["sunnyvale_loop_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":planning_test_base_lib",
        "//modules/planning/planning_component:planning_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
