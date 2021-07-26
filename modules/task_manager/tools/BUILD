load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "routing_record",
    srcs = ["routing_record.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
        "//modules/routing/proto:routing_py_pb2"
    ],
)

py_binary(
    name = "pub_park_go_msg",
    srcs = ["pub_park_go_msg.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/tools/common:proto_utils",
        "//modules/routing/proto:routing_py_pb2",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/task_manager/proto:task_manager_py_pb2"
    ],
)
