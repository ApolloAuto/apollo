load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "video_proto",
    deps = [
        ":sensor_video_proto",
    ],
)

cc_proto_library(
    name = "sensor_video_proto",
    deps = [
        ":video_proto_lib",
    ],
)

proto_library(
    name = "video_proto_lib",
    srcs = ["video_h265cfg.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
