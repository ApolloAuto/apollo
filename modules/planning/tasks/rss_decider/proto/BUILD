load("//tools/proto:proto.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
# load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "rss_decider_proto",
    srcs = ["rss_decider.proto"],
)

apollo_package()
