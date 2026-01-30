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
    name = "libsmooth_stop_trajectory_fallback.so",
    srcs = [
        "smooth_stop_trajectory_fallback.cc",
        "util/speed_planner.cc",
    ],
    hdrs = [
        "smooth_stop_trajectory_fallback.h",
        "util/speed_planner.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_package(enable_source=False)

cpplint()
