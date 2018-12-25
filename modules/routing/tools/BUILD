load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    deps = [
        "//modules/routing:routing_lib",
    ],
)

cc_binary(
    name = "routing_dump",
    srcs = ["routing_dump.cc"],
    deps = [
        "//modules/planning/proto:planning_proto",
        "//modules/routing:routing_lib",
    ],
)

cc_binary(
    name = "routing_cast",
    srcs = ["routing_cast.cc"],
    deps = [
        "//modules/routing:routing_lib",
    ],
)

cpplint()
