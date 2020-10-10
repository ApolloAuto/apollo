load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "xml_parser",
    deps = [
        ":header_xml_parser",
        ":junctions_xml_parser",
        ":lanes_xml_parser",
        ":objects_xml_parser",
        ":roads_xml_parser",
        ":signals_xml_parser",
        ":status",
    ],
)

cc_library(
    name = "common_define",
    hdrs = ["common_define.h"],
    deps = [
        "//cyber/common:log",
        "//modules/map/proto:map_cc_proto",
    ],
)

cc_library(
    name = "header_xml_parser",
    srcs = ["header_xml_parser.cc"],
    hdrs = ["header_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        ":util_xml_parser",
        "//modules/map/hdmap/adapter:coordinate_convert_tool",
        "@com_google_absl//absl/strings",
        "@tinyxml2",
    ],
)

cc_library(
    name = "junctions_xml_parser",
    srcs = ["junctions_xml_parser.cc"],
    hdrs = ["junctions_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        ":util_xml_parser",
        "@tinyxml2",
    ],
)

cc_library(
    name = "lanes_xml_parser",
    srcs = ["lanes_xml_parser.cc"],
    hdrs = ["lanes_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        ":util_xml_parser",
        "@tinyxml2",
    ],
)

cc_library(
    name = "objects_xml_parser",
    srcs = ["objects_xml_parser.cc"],
    hdrs = ["objects_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        ":util_xml_parser",
        "@tinyxml2",
    ],
)

cc_library(
    name = "roads_xml_parser",
    srcs = ["roads_xml_parser.cc"],
    hdrs = ["roads_xml_parser.h"],
    deps = [
        ":common_define",
        ":lanes_xml_parser",
        ":objects_xml_parser",
        ":signals_xml_parser",
        ":status",
        ":util_xml_parser",
        "@tinyxml2",
    ],
)

cc_library(
    name = "signals_xml_parser",
    srcs = ["signals_xml_parser.cc"],
    hdrs = ["signals_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        ":util_xml_parser",
        "@tinyxml2",
    ],
)

cc_library(
    name = "status",
    hdrs = ["status.h"],
    deps = [
        "//modules/common/status",
    ],
)

cc_library(
    name = "util_xml_parser",
    srcs = ["util_xml_parser.cc"],
    hdrs = ["util_xml_parser.h"],
    deps = [
        ":common_define",
        ":status",
        "//cyber/common:log",
        "//modules/map/hdmap/adapter:coordinate_convert_tool",
        "@tinyxml2",
    ],
)

cpplint()
