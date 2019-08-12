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
        "visibility.cc",
        "option_parser.cc",
    ],
    hdrs = [
        "geo_util.h",
        "io_util.h",
		"types.h",
        "macros.h",
        "object.h",
        "object_supplement.h",
        "object_util.h",
        "visibility.h",
        "option_parser.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "@eigen",
        "@pcl",
        "@vtk",
    ],
)

cpplint()
