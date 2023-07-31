load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "model_base",
    srcs = ["libmodel_base.so"],
    hdrs = ["model_base.h"],
)

cc_binary(
    name = "libmodel_base.so",
    srcs = ["model_base.h"],
    linkshared = True,
    linkstatic = True, 
)

apollo_package()
cpplint()