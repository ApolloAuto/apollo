load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "vehicle_model_proto",
    deps = [
        ":vehicle_model_proto_lib",
    ],
)

proto_library(
    name = "vehicle_model_proto_lib",
    srcs = [
        "vehicle_model.proto",
    ],
)

cpplint()
