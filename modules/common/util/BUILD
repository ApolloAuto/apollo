load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")


package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":libutil_tool.so",
        ":libapolloutil.so",
    ],
    deps = [":util_testdata"],
    visibility = ["//visibility:public"],
)

install(
    name = "util_testdata",
    data_dest = "common/addition_data/util",
    data = [
        ":util_test_data",
    ],
)

filegroup(
    name = "util_test_data",
    srcs = glob([
        "testdata/*",
    ]),
)


cc_binary(
    name = "libutil_tool.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":color",
        ":data_extraction",
        ":eigen_defs",
        ":factory",
        ":future",
        ":json_util",
        ":lru_cache",
        ":map_util",
        ":message_util",
        ":perf_util",
        ":string_util",
        ":time_util",
    ],
)

cc_library(
    name = "util_tool",
    srcs = ["libutil_tool.so"],
    hdrs = glob(
        ["*.h"],
        exclude = [
            "point_factory.h",
            "points_downsampler.h",
            "util.h",
        ],
    ),
    deps = [
        "//cyber",
        "@boost",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "future",
    hdrs = ["future.h"],
    alwayslink = True,
    deps = [
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "point_factory",
    hdrs = ["point_factory.h"],
    alwayslink = True,
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = ["util_test.cc"],
    deps = [
        ":util_lib",
        "//modules/common/util/testdata:simple_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "lru_cache",
    hdrs = ["lru_cache.h"],
    alwayslink = True,
)

cc_library(
    name = "color",
    hdrs = ["color.h"],
    alwayslink = True,
)

cc_library(
    name = "data_extraction",
    srcs = ["data_extraction.cc"],
    hdrs = ["data_extraction.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@boost",
    ],
)

cc_library(
    name = "string_util",
    srcs = ["string_util.cc"],
    hdrs = ["string_util.h"],
    alwayslink = True,
    deps = [
        ":future",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "message_util",
    hdrs = ["message_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "message_util_test",
    size = "small",
    srcs = ["message_util_test.cc"],
    tags = [
        "external",
    ],
    deps = [
        ":message_util",
        "//modules/common/util/testdata:simple_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "string_util_test",
    size = "small",
    srcs = ["string_util_test.cc"],
    deps = [
        ":string_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "time_util",
    hdrs = ["time_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "time_util_test",
    size = "small",
    srcs = ["time_util_test.cc"],
    deps = [
        ":time_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "map_util",
    hdrs = ["map_util.h"],
    alwayslink = True,
    deps = [
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "factory_test",
    size = "small",
    srcs = ["factory_test.cc"],
    deps = [
        ":factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lru_cache_test",
    size = "small",
    srcs = ["lru_cache_test.cc"],
    deps = [
        ":lru_cache",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libapolloutil.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":point_factory",
        ":points_downsampler",
        ":util_lib",
    ],
)

cc_library(
    name = "util",
    srcs = ["libapolloutil.so"],
    hdrs = [
        "point_factory.h",
        "points_downsampler.h",
        "util.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "@com_github_gflags_gflags//:gflags",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "points_downsampler",
    hdrs = ["points_downsampler.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/math",
    ],
)

cc_test(
    name = "points_downsampler_test",
    size = "small",
    srcs = ["points_downsampler_test.cc"],
    deps = [
        ":points_downsampler",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "json_util",
    srcs = ["json_util.cc"],
    hdrs = ["json_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "json_util_test",
    size = "small",
    srcs = ["json_util_test.cc"],
    deps = [
        ":json_util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "eigen_defs",
    hdrs = ["eigen_defs.h"],
    alwayslink = True,
    deps = ["@eigen"],
)

cc_library(
    name = "perf_util",
    srcs = ["perf_util.cc"],
    hdrs = ["perf_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "perf_util_test",
    size = "small",
    srcs = ["perf_util_test.cc"],
    deps = [
        ":perf_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
