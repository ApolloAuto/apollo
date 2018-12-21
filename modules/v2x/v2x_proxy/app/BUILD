load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_proxy_lib",
    srcs = [
        "v2x_proxy.cc",
        "main.cc",
    ],
    hdrs = [
        "v2x_proxy.h",
    ],
    deps = [
        "//modules/map/hdmap:hdmap",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface:os_interface",
    ],
)

cpplint()
