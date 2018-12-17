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
    linkopts = [
        "-lPocoFoundation",
    ],
    deps = [
        "//cyber:init",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "class_loader_manager",
    srcs = [
        "class_loader_manager.cc",
    ],
    hdrs = [
        "class_loader_manager.h",
    ],
    deps = [
        "class_loader",
    ],
)

cpplint()
