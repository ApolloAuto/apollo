load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "class_loader_utility",
    srcs = ["class_loader_utility.cc", "class_factory.cc"],
    hdrs = ["class_loader_utility.h", "class_factory.h"],
    deps = [
        "//cyber/class_loader/shared_library",
        "//cyber/common:cyber_common",
    ],
)

apollo_package()
cpplint()
