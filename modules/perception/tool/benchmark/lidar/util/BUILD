load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_util",
    srcs = [
        "geo_util.cc",
        "io_util.cc",
        "object.cc",
        "object_supplement.cc",
        "object_util.cc",
        "option_parser.cc",
        "visibility.cc",
    ],
    hdrs = [
        "geo_util.h",
        "io_util.h",
        "macros.h",
        "object.h",
        "object_supplement.h",
        "object_util.h",
        "option_parser.h",
        "types.h",
        "visibility.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "@com_google_absl//absl/strings",
        "@eigen",
        "@pcl",
        "@vtk",
    ],
)

cpplint()
