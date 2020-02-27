load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "story_proto",
    deps = [
        ":story_proto_lib",
    ],
)

proto_library(
    name = "story_proto_lib",
    srcs = ["story.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "story_pb2",
    src = "story.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)
