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
    deps = [
        "//modules/common",
        "//modules/common/proto:common_proto",
        "//modules/map/proto:map_proto",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/graph",
        "//modules/routing/proto:routing_proto",
    ],
)

cpplint()
