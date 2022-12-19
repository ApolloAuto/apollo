load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

install(
    name = "install",
    library_dest = "map/lib",
    targets = [
        ":libhdmap.so",
        ":libhdmap_util.so"
        ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libhdmap.so",
    srcs = [
        "hdmap.cc",
        "hdmap_common.cc",
        "hdmap_impl.cc",
        "hdmap.h",
        "hdmap_common.h",
        "hdmap_impl.h",
        "hdmap_util.h",
    ],
    linkshared = True,
    linkstatic = True,
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "@com_google_absl//:absl",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "hdmap",
    srcs = ["libhdmap.so"],
    hdrs = [
        "hdmap.h",
        "hdmap_common.h",
        "hdmap_impl.h",
    ],
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap/adapter:opendrive_adapter",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libhdmap_util.so",
    srcs = [
        "hdmap_util.cc",
        "hdmap_util.h",
    ],
    linkshared = True,
    linkstatic = True,
    copts = MAP_COPTS,
    deps = [
        ":hdmap",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common/configs:config_gflags",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "hdmap_util",
    srcs = ["libhdmap_util.so"],
    hdrs = ["hdmap_util.h"],
    copts = MAP_COPTS,
    deps = [
        ":hdmap",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common/configs:config_gflags",
        "@com_google_absl//:absl",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "test-data/*",
    ]),
)

cc_test(
    name = "hdmap_map_test",
    size = "small",
    timeout = "short",
    srcs = [
        "hdmap_common_test.cc",
        "hdmap_impl_test.cc",
    ],
    tags = ["exclude"],
    data = [
        ":testdata",
        #"//modules/common/util",
    ],
    deps = [
        ":hdmap",
        "//cyber",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hdmap_util_test",
    size = "small",
    timeout = "short",
    srcs = ["hdmap_util_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":hdmap_util",
        "@com_google_absl//:absl",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
