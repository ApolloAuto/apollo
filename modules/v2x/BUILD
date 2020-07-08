load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "v2x",
    deps = [
        "//modules/v2x/v2x_proxy/app:v2x_proxy_lib",
    ],
)

cpplint()
