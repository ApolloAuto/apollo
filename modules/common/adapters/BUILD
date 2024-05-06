load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

apollo_cc_library(
    name = "adapter_gflags",
    srcs = ["adapter_gflags.cc"],
    hdrs = ["adapter_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_package()

cpplint()
