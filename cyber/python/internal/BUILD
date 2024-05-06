load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "_cyber_wrapper.so",
    srcs = ["py_cyber.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":py_cyber",
    ],
)

apollo_cc_library(
    name = "py_cyber",
    srcs = ["py_cyber.cc"],
    hdrs = ["py_cyber.h"],
    deps = [
        "//cyber",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)

apollo_cc_test(
    name = "py_cyber_test",
    size = "small",
    srcs = ["py_cyber_test.cc"],
    deps = [
        ":py_cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

apollo_cc_binary(
    name = "_cyber_record_wrapper.so",
    srcs = ["py_record.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":py_record",
    ],
)

apollo_cc_library(
    name = "py_record",
    srcs = ["py_record.cc"],
    hdrs = ["py_record.h"],
    deps = [
        "//cyber",
        "//cyber/message:cyber_message",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)

apollo_cc_test(
    name = "py_record_test",
    size = "small",
    srcs = ["py_record_test.cc"],
    deps = [
        ":py_record",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_binary(
    name = "_cyber_time_wrapper.so",
    srcs = ["py_time.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":py_time",
    ],
)

apollo_cc_library(
    name = "py_time",
    srcs = ["py_time.cc"],
    hdrs = ["py_time.h"],
    deps = [
        "//cyber",
        "@fastrtps",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)

apollo_cc_binary(
    name = "_cyber_timer_wrapper.so",
    srcs = ["py_timer.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":py_timer",
    ],
)

apollo_cc_library(
    name = "py_timer",
    srcs = ["py_timer.cc"],
    hdrs = ["py_timer.h"],
    deps = [
        "//cyber",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)

apollo_cc_binary(
    name = "_cyber_parameter_wrapper.so",
    srcs = ["py_parameter.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":py_parameter",
    ],
)

apollo_cc_library(
    name = "py_parameter",
    srcs = ["py_parameter.cc"],
    hdrs = ["py_parameter.h"],
    deps = [
        ":py_cyber",
        "//cyber",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)

apollo_package()
cpplint()
