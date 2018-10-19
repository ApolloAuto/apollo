load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_common",
    deps = [
      "endpoint",
      "identity",
      "syscall_wrapper",
    ],
)

cc_library(
    name = "endpoint",
    srcs = [ "endpoint.cc", ],
    hdrs = [ "endpoint.h", ],
    deps = [
        "identity",
        "//cyber/common:global_data",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_library(
    name = "identity",
    srcs = [ "identity.cc", ],
    hdrs = [ "identity.h", ],
    deps = [
        "//cyber/common:util",
    ],
)

cc_library(
    name = "syscall_wrapper",
    srcs = [ "syscall_wrapper.cc", ],
    hdrs = [ "syscall_wrapper.h", ],
    deps = [
        "//cyber:binary",
        "//cyber/common:log",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "common_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cpplint()
