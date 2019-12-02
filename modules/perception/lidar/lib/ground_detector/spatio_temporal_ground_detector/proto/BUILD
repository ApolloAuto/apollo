load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "spatio_temporal_ground_detector_config_proto",
    deps = [
        ":spatio_temporal_ground_detector_config_proto_lib",
    ],
)

proto_library(
    name = "spatio_temporal_ground_detector_config_proto_lib",
    srcs = ["spatio_temporal_ground_detector_config.proto"],
)

cpplint()
