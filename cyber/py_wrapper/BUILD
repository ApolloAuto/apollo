load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "_cyber_init.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_init",
    ],
)

cc_library(
    name = "py_init",
    srcs = ["py_init.cc"],
    hdrs = ["py_init.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_init_py3.so",
    linkshared = True,
    deps = [
        ":py3_init",
    ],
)

cc_library(
    name = "py3_init",
    srcs = ["py_init.cc"],
    hdrs = ["py_init.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        "//cyber:cyber_core",
        "@fastrtps",
        "@python3",
    ],
)

cc_test(
    name = "py_init_test",
    srcs = ["py_init_test.cc"],
    deps = [
        ":py3_init",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "_cyber_node.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_node",
    ],
)

cc_library(
    name = "py_node",
    srcs = ["cyber_node_wrap.cc"],
    hdrs = ["py_node.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_record.so",
    linkshared = True,
    deps = [
        ":py_record",
    ],
)

cc_library(
    name = "py_record",
    srcs = ["py_record.cc"],
    hdrs = ["py_record.h"],
    deps = [
        "//cyber/message:py_message",
        "//cyber/record",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_record_py3.so",
    linkshared = True,
    deps = [
        ":py3_record",
    ],
)

cc_library(
    name = "py3_record",
    srcs = ["py_record.cc"],
    hdrs = ["py_record.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        "//cyber/message:py_message",
        "//cyber/record",
        "@python3",
    ],
)

cc_test(
    name = "py_record_test",
    srcs = ["py_record_test.cc"],
    deps = [
        ":py3_record",
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@fastrtps",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "_cyber_time.so",
    linkshared = True,
    deps = [
        ":py_time",
    ],
)

cc_library(
    name = "py_time",
    srcs = ["py_time.cc"],
    hdrs = ["py_time.h"],
    deps = [
        "//cyber:cyber_core",
        "@fastrtps",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_time_py3.so",
    linkshared = True,
    deps = [
        ":py3_time",
    ],
)

cc_library(
    name = "py3_time",
    srcs = ["py_time.cc"],
    hdrs = ["py_time.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        "//cyber:cyber_core",
        "@fastrtps",
        "@python3",
    ],
)

cc_binary(
    name = "_cyber_timer.so",
    linkshared = True,
    deps = [
        ":py_timer",
    ],
)

cc_library(
    name = "py_timer",
    srcs = ["py_timer.cc"],
    hdrs = ["py_timer.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_timer_py3.so",
    linkshared = True,
    deps = [
        ":py3_timer",
    ],
)

cc_library(
    name = "py3_timer",
    srcs = ["py_timer.cc"],
    hdrs = ["py_timer.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        "//cyber:cyber_core",
        "@python3",
    ],
)

cc_binary(
    name = "_cyber_parameter.so",
    linkshared = True,
    deps = [
        ":py_parameter",
    ],
)

cc_library(
    name = "py_parameter",
    srcs = ["cyber_parameter_wrap.cc"],
    hdrs = [
        "py_node.h",
        "py_parameter.h",
    ],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cpplint()
