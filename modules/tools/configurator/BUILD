load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "configurator",
    srcs = ["configurator.py"],
    data = ["meta.data"],
    deps = [
        ":ModuleConf",
    ],
)

py_library(
    name = "ModuleConf",
    srcs = ["ModuleConf.py"],
)
