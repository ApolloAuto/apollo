package(default_visibility = ["//visibility:public"])

load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "perception_proxy",
    srcs = [
        "perception_proxy.cc",
    ],
    hdrs = [
        "perception_proxy.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
