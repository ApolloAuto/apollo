load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "yolo_proto",
    deps = [
        ":yolo_proto_lib",
    ],
)

proto_library(
    name = "yolo_proto_lib",
    srcs = ["yolo.proto"],
)

cpplint()
