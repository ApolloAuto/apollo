load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util_lib",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/routing:routing_lib",
    ],
)

cpplint()
