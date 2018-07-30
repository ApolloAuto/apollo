load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

genrule(
    name = "cuda_util",
    srcs = [
        "cuda_util.cu",
        "cuda_util.h",
    ],
    outs = ["libcuda_util.so"],
    cmd = """
    nvcc \
    -arch=sm_52 \
    -gencode=arch=compute_50,code=sm_50 \
    -gencode=arch=compute_52,code=sm_52 \
    -gencode=arch=compute_60,code=sm_60 \
    -gencode=arch=compute_61,code=sm_61 \
    -gencode=arch=compute_61,code=compute_61 \
    -std=c++11 \
    -lcudart -lcublas \
    -I /apollo \
    --shared \
    --compiler-options '-fPIC' \
    --expt-extended-lambda \
    $(location cuda_util.cu) \
    -o $@
     """,
)

cc_test(
    name = "cuda_util_test",
    size = "small",
    srcs = [
        "cuda_util.h",
        "cuda_util_test.cc",
        "libcuda_util.so",
    ],
    deps = [
        "//modules/common/math",
        "@cuda",
        "@gtest//:main",
    ],
)

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
