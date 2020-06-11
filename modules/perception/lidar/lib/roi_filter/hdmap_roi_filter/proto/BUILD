load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "hdmap_roi_filter_proto",
    deps = [
        ":hdmap_roi_filter_proto_lib",
    ],
)

proto_library(
    name = "hdmap_roi_filter_proto_lib",
    srcs = ["hdmap_roi_filter.proto"],
)

py_proto(
    name = "hdmap_roi_filter_pb2",
    src = "hdmap_roi_filter.proto",
)
