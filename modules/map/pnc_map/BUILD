load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path",
    srcs = [
        "path.cc",
    ],
    hdrs = [
        "path.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_proto",
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "route_segments",
    srcs = [
        "route_segments.cc",
    ],
    hdrs = [
        "route_segments.h",
    ],
    deps = [
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/map/hdmap",
        "//modules/map/pnc_map:path",
        "//modules/routing/common:routing_gflags",
    ],
)

cc_library(
    name = "pnc_map",
    srcs = [
        "pnc_map.cc",
    ],
    hdrs = [
        "pnc_map.h",
    ],
    deps = [
        ":path",
        ":route_segments",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_gflags",
        "//modules/routing/common:routing_gflags",
        "@gtest//:gtest",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_test(
    name = "pnc_path_test",
    size = "small",
    srcs = [
        "path_test.cc",
    ],
    deps = [
        ":path",
        "//modules/common:log",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pnc_map_test",
    size = "small",
    srcs = [
        "pnc_map_test.cc",
    ],
    data = [
        ":testdata",
        "//modules/map:map_data",
    ],
    deps = [
        ":pnc_map",
        "//modules/common:log",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "route_segments_test",
    size = "small",
    srcs = [
        "route_segments_test.cc",
    ],
    data = [
        ":testdata",
        "//modules/map:map_data",
    ],
    deps = [
        ":route_segments",
        "//modules/common:log",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
