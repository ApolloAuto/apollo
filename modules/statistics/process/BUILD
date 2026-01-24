load("//tools:apollo_package.bzl", "apollo_package", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"process_statistic\\"']

apollo_component(
    name = "libprocess_statistic_component.so",
    srcs = [
        "process_statistic_component.cc",
        "process_statistic_gflags.cc",
    ],
    hdrs = [
        "process_statistic_component.h",
        "process_statistic_gflags.h",
    ],
    copts = COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/statistics_msgs:process_statistic_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
        "dag/*",
    ]),
)

apollo_package(enable_source=False)
cpplint()
