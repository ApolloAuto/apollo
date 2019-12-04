load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_proxy_lib",
    srcs = [
        "main.cc",
        "v2x_proxy.cc",
    ],
    hdrs = ["v2x_proxy.h"],
    deps = [
        "//modules/map/hdmap",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface",
    ],
)

cpplint()
