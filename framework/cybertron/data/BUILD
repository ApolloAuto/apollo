load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_lib",
    srcs = glob([
        "*.cpp",
        "**/*.cpp",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron/proto:component_config_cc_proto",
    ],        
)

cpplint()
