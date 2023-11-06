load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "mock_routing_request",
    srcs = ["mock_routing_request.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/mock_routing",
#     targets = [
#         ":mock_routing_request",
#     ]
# )
apollo_package()