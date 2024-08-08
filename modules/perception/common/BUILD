load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "perception_gflags",
    srcs = ["perception_gflags.cc"],
    hdrs = ["perception_gflags.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_cc_library(
    name = "perception_common_util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = ["//cyber"],
)

apollo_package()
cpplint()
