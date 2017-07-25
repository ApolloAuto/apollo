load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_point_util",
    srcs = [
        "path_point_util.cc",
        "sl_point_util.cc",
    ],
    hdrs = [
        "path_point_util.h",
        "sl_point_util.h",
    ],
    deps = [
        "//modules/common/proto:path_point_proto",
        "//modules/planning/math:hermite_spline",
        "//modules/common/math:math",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "path",
    hdrs = [
        "path.h",
    ],
)

cc_library(
    name = "discretized_path",
    srcs = [
        "discretized_path.cc",
    ],
    hdrs = [
        "discretized_path.h",
    ],
    deps = [
        ":path",
        ":path_point_util",
        "//modules/common/proto:path_point_proto",
    ],
)

cc_library(
    name = "frenet_frame_path",
    srcs = [
        "frenet_frame_path.cc",
    ],
    hdrs = [
        "frenet_frame_path.h",
    ],
    deps = [
        ":path",
        ":path_point_util",
        "//modules/common/proto:path_point_proto",
    ],
)

cc_library(
    name = "path_data",
    srcs = [
        "path_data.cc",
    ],
    hdrs = [
        "path_data.h",
    ],
    deps = [
        ":discretized_path",
        ":frenet_frame_path",
        "//modules/planning/math:double",
    ],
)

cpplint()
