load("//tools:cpplint.bzl", "cpplint")

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

cpplint()
