load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_cc_test(
    name = "open_space_fallback_decider_test",
    size = "small",
    srcs = ["open_space_fallback_decider_test.cc"],
    deps = [
        ":open_space_fallback_decider_lib",
        "@com_google_googletest//:gtest_main",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_plugin(
    name = "libopen_space_fallback_decider.so",
    srcs = ["open_space_fallback_decider.cc"],
    hdrs = ["open_space_fallback_decider.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/open_space_fallback_decider/proto:open_space_fallback_decider_cc_proto",
    ],
)

apollo_package()

cpplint()
