load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "client_wrapper",
    hdrs = ["client_wrapper.h"],
    deps = [
        "//cyber",
    ],
)

cpplint()

apollo_package()
