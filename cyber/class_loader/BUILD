load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "class_loader",
    srcs = [
        "class_loader.cc",
        "utility/class_factory.cc",
        "utility/class_loader_utility.cc",
    ],
    hdrs = [
        "class_loader.h",
        "class_loader_register_macro.h",
        "utility/class_factory.h",
        "utility/class_loader_utility.h",
    ],
    deps = [
        "//cyber:init",
        "//cyber/common:log",
        "//third_party:PocoFoundation",
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

cpplint()
