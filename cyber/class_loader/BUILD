load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "cyber_class_loader_hdrs",
    srcs = glob([
        "*.h",
    ]),
)

cc_library(
    name = "class_loader",
    srcs = ["class_loader.cc"],
    hdrs = ["class_loader.h"],
    alwayslink = True,
    deps = [
        ":class_loader_register_macro",
        "//cyber/class_loader/shared_library",
        "//cyber/class_loader/utility:class_factory",
        "//cyber/class_loader/utility:class_loader_utility",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "class_loader_register_macro",
    hdrs = ["class_loader_register_macro.h"],
    deps = [
        "//cyber/class_loader/utility:class_loader_utility",
    ],
)

cc_library(
    name = "class_loader_manager",
    srcs = ["class_loader_manager.cc"],
    hdrs = ["class_loader_manager.h"],
    alwayslink = True,
    deps = [
        ":class_loader",
    ],
)

cc_test(
    name = "class_loader_test",
    size = "small",
    srcs = ["class_loader_test.cc"],
    data = [
        "//cyber/class_loader/test:plugin1",
        "//cyber/class_loader/test:plugin2",
    ],
    deps = [
        "//cyber",
        "//cyber/class_loader/test:base",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
