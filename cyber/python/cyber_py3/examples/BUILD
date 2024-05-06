load("@rules_python//python:defs.bzl", "py_binary")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# py_binary(
#     name = "py_record",
#     srcs = ["py_record.py"],
#     deps = [
#         "//cyber/proto:unit_test_py_pb2",
#         "//cyber/python/cyber_py3:record",
#         "//modules/common_msgs/basic_msgs:error_code_py_pb2",
#         "//modules/common_msgs/basic_msgs:header_py_pb2",
#         "//modules/common/util/testdata:simple_py_pb2",
#     ],
# )

py_binary(
    name = "py_time",
    srcs = ["py_time.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
    ],
)

py_binary(
    name = "py_timer",
    srcs = ["py_timer.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_timer",
    ],
)

py_binary(
    name = "py_talker",
    srcs = ["py_talker.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "py_listener",
    srcs = ["py_listener.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "py_parameter",
    srcs = ["py_parameter.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:parameter",
    ],
)

py_binary(
    name = "py_service",
    srcs = ["py_service.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "py_client",
    srcs = ["py_client.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "py_record_trans",
    srcs = ["py_record_trans.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)

py_binary(
    name = "py_record_channel_info",
    srcs = ["py_record_channel_info.py"],
    deps = [
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)

apollo_package()