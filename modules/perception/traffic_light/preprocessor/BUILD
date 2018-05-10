load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_preprocessor",
    srcs = [
        "tl_preprocessor.cc",
    ],
    hdrs = [
        "tl_preprocessor.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
        "//modules/perception/traffic_light/projection:perception_traffic_light_projection",
    ],
)

cpplint()
