load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

gpu_library(
    name = "cuda_pnc_util",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/math:geometry",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_test(
    name = "cuda_util_test",
    size = "small",
    srcs = [
        "cuda_util.h",
        "cuda_util_test.cc",
        ":cuda_pnc_util",
    ],
    deps = [
        "//modules/common/math",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hipblas",
        "@local_config_rocm//rocm:rocm_headers",
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "path",
    srcs = ["path.cc"],
    hdrs = ["path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_lane_cc_proto",
    ],
)

cc_library(
    name = "route_segments",
    srcs = ["route_segments.cc"],
    hdrs = ["route_segments.h"],
    copts = MAP_COPTS,
    deps = [
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/map/hdmap",
        "//modules/map/pnc_map:path",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_cc_proto",
    ],
)

cc_library(
    name = "pnc_map",
    srcs = ["pnc_map.cc"],
    hdrs = ["pnc_map.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":path",
        ":route_segments",
        "//modules/common/util:point_factory",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_gflags",
        "//modules/routing/common:routing_gflags",
        "@com_google_googletest//:gtest",
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
    srcs = ["path_test.cc"],
    deps = [
        ":path",
        "//modules/common/util",
        "//modules/routing/proto:routing_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pnc_map_test",
    size = "small",
    srcs = ["pnc_map_test.cc"],
    data = [
        ":testdata",
        "//modules/map/data:map_sunnyvale_loop",
    ],
    deps = [
        ":pnc_map",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "route_segments_test",
    size = "small",
    srcs = ["route_segments_test.cc"],
    data = [
        ":testdata",
        "//modules/map/data:map_sunnyvale_loop",
    ],
    deps = [
        ":route_segments",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
