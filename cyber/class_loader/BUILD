load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "class_loader",
    srcs = [
        "class_loader.cc",
    ],
    hdrs = [
        "class_loader.h",
    ],
    deps = [
        ":class_factory",
        ":class_loader_utility",
    ],
)

cc_library(
    name = "class_loader_manager",
    srcs = ["class_loader_manager.cc"],
    hdrs = ["class_loader_manager.h"],
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

cc_library(
    name = "class_factory",
    srcs = ["class_factory.cc"],
    hdrs = [
        "class_loader.h",
        "class_loader_register_macro.h",
        "class_factory.h",
        "class_loader_utility.h",
    ],
    deps = [
        "//cyber:init",
        "@poco//:PocoFoundation",
    ],
)

cc_library(
    name = "class_loader_utility",
    srcs = [
        "class_loader_utility.cc",
    ],
    hdrs = [
        "class_loader_utility.h",
    ],
    deps = [
        "//cyber:init",
        "//cyber/common:log",
        "@poco//:PocoFoundation",
        ":class_factory"
    ],
)

cpplint()
