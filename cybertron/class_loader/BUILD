load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "class_loader_lib",
    srcs = glob([
        "class_factory.cpp",
        "class_loader.cpp",
        "class_loader_manager.cpp",
        "class_loader_utility.cpp",
    ]),
    hdrs = glob([
        "class_factory.h",
        "class_loader.h",
        "class_loader_manager.h",
        "class_loader_register_macro.h",
        "class_loader_utility.h",
    ]),
    deps = [
          "//cybertron/common:common",
    ],
    linkopts = [
        "-lPocoFoundation",
    ],
)

cpplint()
