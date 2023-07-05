load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
    ],
)

py_binary(
    name = "mapshow",
    srcs = ["mapshow.py"],
    deps = [
        "//modules/tools/mapshow/libs:localization",
        "//modules/tools/mapshow/libs:map",
        "//modules/tools/mapshow/libs:path",
    ],
)

py_binary(
    name = "roadshow",
    srcs = ["roadshow.py"],
    deps = [
        "//modules/tools/mapshow/libs:map",
    ],
)

install(
    name = "install",
    data = [":readme"],
    data_dest = "tools/mapshow",
    py_dest = "tools/mapshow",
    targets = [
        "mapshow",
        "roadshow",
    ]
)
