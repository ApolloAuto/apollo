load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_common_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/common:common_lib",
        "//cybertron/proto:role_attributes_cc_proto",
    ],
)

cpplint()
