load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "conti_ars_preprocessor",
    srcs = [
        "conti_ars_preprocessor.cc",
    ],
    hdrs = [
        "conti_ars_preprocessor.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/lib/utils:utils",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:radar_lib_interface",
        "//modules/drivers/proto:sensor_proto",
        "@eigen",
    ],
)

cpplint()
