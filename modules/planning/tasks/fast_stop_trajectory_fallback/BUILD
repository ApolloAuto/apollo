load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libfast_stop_trajectory_fallback.so",
    srcs = ["fast_stop_trajectory_fallback.cc"],
    hdrs = ["fast_stop_trajectory_fallback.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_package()

cpplint()
