load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "py_talker",
    srcs = ["py_talker.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "//cyber/py_wrapper:py_cyber",
    ],
)

cc_binary(
    name = "py_listener",
    srcs = ["py_listener.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "//cyber/py_wrapper:py_cyber",
    ],
)

cc_binary(
    name = "py_service",
    srcs = ["py_service.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "//cyber/py_wrapper:py_cyber",
    ],
)

cpplint()
