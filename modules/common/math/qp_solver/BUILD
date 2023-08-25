load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "qp_solver",
    srcs = ["qp_solver.cc"],
    hdrs = ["qp_solver.h"],
    deps = [
        "@eigen",
    ],
)

apollo_package()

cpplint()
