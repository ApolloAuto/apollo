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
    srcs = ["cyber_init_wrap.cc"],
    hdrs = ["py_init.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
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
    srcs = ["cyber_record_wrap.cc"],
    hdrs = ["py_record.h"],
    deps = [
        "//cyber/message:py_message",
        "//cyber/record",
        "@python27",
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
    srcs = ["cyber_time_wrap.cc"],
    hdrs = ["py_time.h"],
    deps = [
        "//cyber:cyber_core",
        "@fastrtps",
        "@python27",
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
    srcs = ["cyber_timer_wrap.cc"],
    hdrs = ["py_timer.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
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
