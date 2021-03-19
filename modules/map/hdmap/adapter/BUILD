load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opendrive_adapter",
    srcs = [
        "opendrive_adapter.cc",
    ],
    hdrs = [
        "opendrive_adapter.h",
    ],
    deps = [
        ":coordinate_convert_tool",
        ":proto_organizer",
        "//cyber/common:log",
        "//modules/map/hdmap/adapter/xml_parser",
    ],
)

cc_library(
    name = "coordinate_convert_tool",
    srcs = ["coordinate_convert_tool.cc"],
    hdrs = ["coordinate_convert_tool.h"],
    deps = [
        "//modules/map/hdmap/adapter/xml_parser:status",
        "@com_github_google_glog//:glog",
        "@proj",
    ],
)

cc_library(
    name = "proto_organizer",
    srcs = ["proto_organizer.cc"],
    hdrs = ["proto_organizer.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/map/hdmap/adapter/xml_parser:common_define",
        "//modules/map/proto:map_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
