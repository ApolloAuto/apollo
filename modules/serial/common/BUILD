load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
    deps = [
        "//cyber",
    ],
)

apollo_cc_library(
    name = "serial_stream",
    hdrs = [
        "serial_stream.h",
    ],
    srcs = [
        "serial_stream.cc",
    ],
    deps = [
        "//cyber",
    ],
)

apollo_package()
cpplint()
