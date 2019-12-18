load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "scene_manager_config_proto",
    deps = [
        ":scene_manager_config_proto_lib",
    ],
)

proto_library(
    name = "scene_manager_config_proto_lib",
    srcs = ["scene_manager_config.proto"],
)

cpplint()
