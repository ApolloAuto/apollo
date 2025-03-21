load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "paddle_net",
  srcs = ["paddle_net.cc"],
  hdrs = ["paddle_net.h"],
  deps = [
    "//modules/perception/inference:inference_lib",
  ]+ select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"],
    }),
)

cpplint()
