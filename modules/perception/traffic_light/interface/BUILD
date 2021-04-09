load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    srcs = [
        "base_projection.cc",
    ],
    hdrs = [
        "base_preprocessor.h",
        "base_projection.h",
        "base_recognizer.h",
        "base_rectifier.h",
        "base_reviser.h",
        "green_interface.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base",
    ],
)

cpplint()
