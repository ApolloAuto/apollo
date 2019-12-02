load("//tools:cpplint.bzl", "cpplint")

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

cpplint()
