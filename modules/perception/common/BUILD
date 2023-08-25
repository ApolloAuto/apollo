load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "perception_gflags",
    srcs = ["perception_gflags.cc"],
    hdrs = ["perception_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_cc_library(
    name = "perception_common_util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = ["//cyber"],
)

apollo_package()
cpplint()
