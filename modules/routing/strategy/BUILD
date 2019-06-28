load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "strategy",
    deps = [
        ":routing_a_star_strategy",
    ],
)

cc_library(
    name = "routing_a_star_strategy",
    srcs = [
        "a_star_strategy.cc",
    ],
    hdrs = [
        "a_star_strategy.h",
        "strategy.h",
    ],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/graph",
    ],
)

cpplint()
