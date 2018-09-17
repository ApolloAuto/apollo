load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    deps = [
        "//modules/common",
        "//modules/common/util",
        "//modules/routing:routing_lib",
        "//framework:cybertron",
   ],
)

cc_binary(
    name = "routing_dump",
    srcs = ["routing_dump.cc"],
    deps = [
        "//modules/common",
        "//modules/common/util",
        "//modules/planning/proto:planning_proto",
        "//modules/routing:routing_lib",
        "//framework:cybertron",
   ],
)

cpplint()
