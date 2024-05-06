load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "util_test_data",
    srcs = glob([
        "testdata/*",
    ]),
)

apollo_cc_library(
    name = "util_tool",
    srcs = [
        "data_extraction.cc",
        "json_util.cc",
        "perf_util.cc",
        "string_util.cc",
    ],
    hdrs = [
        "color.h",
        "eigen_defs.h",
        "data_extraction.h",
        "factory.h",
        "future.h",
        "json_util.h",
        "lru_cache.h",
        "map_util.h",
        "perf_util.h",
        "string_util.h",
        "time_util.h",
    ],
    deps = [
        "//cyber",
        ":message_util",
        "@boost",
        "@eigen",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

apollo_cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "util_test",
    size = "small",
    srcs = ["util_test.cc"],
    deps = [
        ":util_lib",
        "//modules/common/util/testdata:simple_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "message_util",
    hdrs = ["message_util.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

apollo_cc_test(
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

apollo_cc_test(
    name = "string_util_test",
    size = "small",
    srcs = ["string_util_test.cc"],
    deps = [
        ":util_tool",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "time_util_test",
    size = "small",
    srcs = ["time_util_test.cc"],
    deps = [
        ":util_tool",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "factory_test",
    size = "small",
    srcs = ["factory_test.cc"],
    deps = [
        ":util_tool",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lru_cache_test",
    size = "small",
    srcs = ["lru_cache_test.cc"],
    deps = [
        ":util_tool",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "common_util",
    hdrs = [
        "point_factory.h",
        "points_downsampler.h",
        "util.h",
    ],
    srcs = [
        "util.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "points_downsampler_test",
    size = "small",
    srcs = ["points_downsampler_test.cc"],
    deps = [
        ":common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "json_util_test",
    size = "small",
    srcs = ["json_util_test.cc"],
    deps = [
        ":util_tool",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "perf_util_test",
    size = "small",
    srcs = ["perf_util_test.cc"],
    deps = [
        ":util_tool",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
