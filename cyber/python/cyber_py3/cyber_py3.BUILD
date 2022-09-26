load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

pb_deps = ["@com_google_protobuf//:protobuf_python"]

py_library(
    name = "cyber_time",
    srcs = ["cyber_time.py"],
    data = [
        "//python/internal:_cyber_time_wrapper.so",
        "//python/internal:_cyber_wrapper.so",
    ],
    deps = pb_deps
)

py_library(
    name = "cyber_timer",
    srcs = ["cyber_timer.py"],
    data = [
        "//python/internal:_cyber_timer_wrapper.so",
    ],
    deps = pb_deps
)

py_library(
    name = "cyber",
    srcs = ["cyber.py"],
    data = [
        "//python/internal:_cyber_wrapper.so",
    ],
    deps = pb_deps
)

py_library(
    name = "parameter",
    srcs = ["parameter.py"],
    data = [
        "//python/internal:_cyber_parameter_wrapper.so",
    ],
    deps = pb_deps
)

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//python/internal:_cyber_record_wrapper.so",
    ],
    deps = pb_deps
)
