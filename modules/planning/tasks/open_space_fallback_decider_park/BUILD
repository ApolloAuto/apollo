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

apollo_plugin(
    name = "libopen_space_fallback_decider_park.so",
    srcs = ["open_space_fallback_decider_park.cc"],
    hdrs = ["open_space_fallback_decider_park.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/tasks/open_space_fallback_decider_park/proto:open_space_fallback_decider_park_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
