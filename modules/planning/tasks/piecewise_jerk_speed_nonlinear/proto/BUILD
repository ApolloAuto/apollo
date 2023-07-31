load("//tools/proto:proto.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
# load("//tools/install:install.bzl", "install", "install_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "piecewise_jerk_speed_nonlinear_proto",
    srcs = ["piecewise_jerk_speed_nonlinear.proto"],
)

apollo_package()
