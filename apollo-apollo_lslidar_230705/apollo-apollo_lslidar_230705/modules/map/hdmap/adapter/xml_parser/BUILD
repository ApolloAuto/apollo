load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")


cc_library(
    name = "xml_parser",
    deps = [
        ":common_define",
        ":coordinate_convert_tool",
        ":header_xml_parser",
        ":junctions_xml_parser",
        ":lanes_xml_parser",
        ":objects_xml_parser",
        ":roads_xml_parser",
        ":signals_xml_parser",
        ":status",
    ],
    visibility = [
        "//visibility:public",
    ],
)

cc_library(
    name = "common_define",
    hdrs = ["common_define.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
    ],
)

cc_library(
    name = "coordinate_convert_tool",
    srcs = ["coordinate_convert_tool.cc"],
    hdrs = ["coordinate_convert_tool.h"],
    alwayslink = True,
    deps = [
        ":status",
        "@com_github_google_glog//:glog",
        "@proj",
    ],
)

cc_library(
    name = "header_xml_parser",
    srcs = ["header_xml_parser.cc"],
    hdrs = ["header_xml_parser.h"],
    alwayslink = True,
    deps = [
        ":common_define",
        ":coordinate_convert_tool",
        ":status",
        ":util_xml_parser",
        "@com_google_absl//:absl",
        "@tinyxml2",
    ],
)

cc_library(
    name = "junctions_xml_parser",
    srcs = ["junctions_xml_parser.cc"],
    hdrs = ["junctions_xml_parser.h"],
    alwayslink = True,
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
    alwayslink = True,
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
    alwayslink = True,
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
    alwayslink = True,
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
    alwayslink = True,
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
    alwayslink = True,
    deps = [
        "//modules/common/status",
    ],
)

cc_library(
    name = "util_xml_parser",
    srcs = ["util_xml_parser.cc"],
    hdrs = ["util_xml_parser.h"],
    alwayslink = True,
    deps = [
        ":common_define",
        ":coordinate_convert_tool",
        ":status",
        "//cyber",
        "@tinyxml2",
    ],
)

cpplint()
