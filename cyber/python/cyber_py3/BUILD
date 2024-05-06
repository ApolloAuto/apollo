load("@rules_python//python:defs.bzl", "py_library")
# load("//tools/install:install.bzl", "install", "install_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# filegroup(
#     name = "runtime_files",
#     srcs = glob([
#         "*.py",
#     ])
# )

# install(
#     name = "cyber_python_library",
#     data = [":runtime_files"],
#     data_dest = "cyber/python/cyber/python/cyber_py3"
# )

py_library(
    name = "cyber_time",
    srcs = ["cyber_time.py"],
    data = [
        "//cyber/python/internal:_cyber_time_wrapper.so",
        "//cyber/python/internal:_cyber_wrapper.so",
    ],
)

py_library(
    name = "cyber_timer",
    srcs = ["cyber_timer.py"],
    data = [
        "//cyber/python/internal:_cyber_timer_wrapper.so",
    ],
)

py_library(
    name = "cyber",
    srcs = ["cyber.py"],
    data = [
        "//cyber/python/internal:_cyber_wrapper.so",
    ],
)

py_library(
    name = "parameter",
    srcs = ["parameter.py"],
    data = [
        "//cyber/python/internal:_cyber_parameter_wrapper.so",
    ],
)

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//cyber/python/internal:_cyber_record_wrapper.so",
    ],
)

apollo_package()