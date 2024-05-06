load("//tools/proto:proto.bzl", "apollo_py_binary")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "viewer_raw",
    srcs = ["viewer_raw.py"],
)

apollo_py_binary(
    name = "viewer_smooth",
    srcs = ["viewer_smooth.py"],
)

apollo_py_binary(
    name = "record_extractor",
    srcs = ["record_extractor.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_package()