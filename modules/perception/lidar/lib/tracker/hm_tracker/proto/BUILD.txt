load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "hm_tracker_config_proto",
    deps = [
        ":hm_tracker_config_proto_lib",
    ],
)

proto_library(
    name = "hm_tracker_config_proto_lib",
    srcs = [
        "hm_tracker_config.proto",
    ],
    deps = [
    ],
)

cpplint()
