load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "_cyber.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_cyber",
    ],
)

cc_library(
    name = "py_cyber",
    srcs = ["py_cyber.cc"],
    hdrs = ["py_cyber.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_py3.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py3_cyber",
    ],
)

cc_library(
    name = "py3_cyber",
    srcs = ["py_cyber.cc"],
    hdrs = ["py_cyber.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        "//cyber:cyber_core",
        "//third_party:python3",
    ],
)

cc_test(
    name = "py_cyber_test",
    srcs = ["py_cyber_test.cc"],
    deps = [
        ":py3_cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
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
        "//third_party:python3",
    ],
)

cc_test(
    name = "py_record_test",
    srcs = ["py_record_test.cc"],
    deps = [
        ":py3_record",
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@fastrtps",
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
        "//third_party:python3",
        "@fastrtps",
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
        "//third_party:python3",
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
    srcs = ["py_parameter.cc"],
    hdrs = ["py_parameter.h"],
    deps = [
        ":py_cyber",
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_parameter_py3.so",
    linkshared = True,
    deps = [
        ":py3_parameter",
    ],
)

cc_library(
    name = "py3_parameter",
    srcs = ["py_parameter.cc"],
    hdrs = ["py_parameter.h"],
    defines = ["PY_MAJOR_VERSION=3"],
    deps = [
        ":py3_cyber",
        "//cyber:cyber_core",
        "//third_party:python3",
    ],
)

cpplint()
