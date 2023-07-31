load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "client_wrapper",
    hdrs = ["client_wrapper.h"],
    deps = [
        "//cyber/service:client",
    ],
)

cpplint()
apollo_package()
