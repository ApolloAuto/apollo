load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_routing",
    srcs = [
        "routing.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/monitor",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "//modules/routing/common",
        "//modules/routing/core",
        "//modules/routing/proto:routing_proto",
        "//modules/common/status",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "routing",
    srcs = ["main.cc"],
    deps = [
        ":lib_routing",
        "//external:gflags",
        "//modules/routing/common",
    ],
)

cpplint()
