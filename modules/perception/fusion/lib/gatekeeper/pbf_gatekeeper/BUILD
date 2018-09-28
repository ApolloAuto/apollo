load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_gatekeeper",
    srcs = [
        "pbf_gatekeeper.cc",
    ],
    hdrs = [
        "pbf_gatekeeper.h",
    ],
    deps = [
        "//modules/perception/fusion/lib/interface:interface",
        "//modules/perception/fusion/base:base_init_options",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/proto:pbf_gatekeeper_config_proto",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:track",
    ],
)

cpplint()
