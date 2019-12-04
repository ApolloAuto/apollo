load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "object_template_meta_schema_proto",
    deps = [
        ":object_template_meta_schema_proto_lib",
    ],
)

proto_library(
    name = "object_template_meta_schema_proto_lib",
    srcs = ["object_template_meta_schema.proto"],
)

cpplint()
