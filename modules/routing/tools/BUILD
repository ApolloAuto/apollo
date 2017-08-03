load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "topo_creator_main",
    srcs = ["topo_creator_main.cc"],
    deps = [
        "//external:gflags",
        "//modules/routing/topo_creator",
        "//modules/common:log",
        "//modules/routing/common"
    ],
)

cpplint()

