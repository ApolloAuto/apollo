load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    testonly = True,
    hdrs = ["base.h"],
)

cc_library(
    name = "plugin1",
    testonly = True,
    srcs = ["plugin1.cc"],
    deps = [
        ":base",
        "//cyber/class_loader:class_loader_manager",
    ],
)

cc_library(
    name = "plugin2",
    testonly = True,
    srcs = ["plugin2.cc"],
    deps = [
        ":base",
        "//cyber/class_loader:class_loader_manager",
    ],
)

cpplint()
