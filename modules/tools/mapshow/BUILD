load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

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
