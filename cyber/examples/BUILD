load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

apollo_cc_binary(
    name = "listener",
    srcs = ["listener.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

apollo_cc_binary(
    name = "paramserver",
    srcs = ["paramserver.cc"],
    deps = [
        "//cyber",
    ],
)

apollo_cc_binary(
    name = "service",
    srcs = ["service.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

apollo_cc_binary(
    name = "record",
    srcs = ["record.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
    ],
)

apollo_package()
cpplint()
