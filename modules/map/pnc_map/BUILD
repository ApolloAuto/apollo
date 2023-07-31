load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cuda_library(
    name = "cuda_pnc_util",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "@local_config_cuda//cuda:cudart",
    ],
    tags = ["exclude"]
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
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ],
    tags = ["exclude"],
    linkstatic = True,
)

cc_library(
    name = "path",
    srcs = [":libpath.so"],
    hdrs = ["path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap",
        "@com_google_absl//:absl",
    ],
)

cc_binary(
    name = "libpath.so",
    srcs = ["path.cc", "path.h"],
    linkshared = True,
    linkstatic = True,
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap",
        "@com_google_absl//:absl",
    ],
)

cc_binary(
    name = "libroute_segments.so",
    srcs = ["route_segments.cc", "route_segments.h"],
    linkshared = True,
    linkstatic = True,
    copts = MAP_COPTS,
    deps = [
        ":path",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/util",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/map/hdmap",
    ],
)

cc_library(
    name = "route_segments",
    srcs = [":libroute_segments.so"],
    hdrs = ["route_segments.h"],
    copts = MAP_COPTS,
    alwayslink = True,
    deps = [
        ":path",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/util",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/map/hdmap",
    ],
)

cc_binary(
    name = "libpnc_map.so",
    srcs = [
        "pnc_map_base.cc",
        "pnc_map_base.h",
        "path.h",
    ],
    linkshared = True,
    linkstatic = True,
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":route_segments",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common/util",
        "//modules/common/configs:config_gflags",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
    ],
)

cc_library(
    name = "pnc_map",
    srcs = [":libpnc_map.so"],
    hdrs = [
        "pnc_map_base.h",
        "path.h",
        "route_segments.h",
    ],
    alwayslink = True,
    linkstatic = True,
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":route_segments",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common/util",
        "//modules/common/configs:config_gflags",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
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
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "route_segments_test",
    size = "small",
    srcs = ["route_segments_test.cc"],
    data = [
        ":testdata",
        "//modules/map:map_data",
    ],
    deps = [
        ":route_segments",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
