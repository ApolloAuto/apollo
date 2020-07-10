load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "mock_routing_request",
    srcs = ["mock_routing_request.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/routing/proto:routing_py_pb2",
    ],
)
