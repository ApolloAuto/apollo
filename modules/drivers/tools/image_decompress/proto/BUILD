load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decompress_proto",
    deps = [
        ":image_decompress_proto",
    ],
)

cc_proto_library(
    name = "image_decompress_proto",
    deps = [
        ":image_decompress_proto_lib",
    ],
)

proto_library(
    name = "image_decompress_proto_lib",
    srcs = ["config.proto"],
)

cpplint()
