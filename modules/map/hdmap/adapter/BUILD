load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opendrive_adapter",
    srcs = [
        "xml_parser/header_xml_parser.cc",
        "xml_parser/objects_xml_parser.cc",
        "xml_parser/lanes_xml_parser.cc",
        "xml_parser/junctions_xml_parser.cc",
        "xml_parser/roads_xml_parser.cc",
        "xml_parser/signals_xml_parser.cc",
        "xml_parser/util_xml_parser.cc",
        "coordinate_convert_tool.cc",
        "proto_reorganization.cc",
    ],
    hdrs = [
        "xml_parser/common_define.h",
        "xml_parser/status.h",
        "xml_parser/header_xml_parser.h",
        "xml_parser/objects_xml_parser.h",
        "xml_parser/lanes_xml_parser.h",
        "xml_parser/junctions_xml_parser.h",
        "xml_parser/roads_xml_parser.h",
        "xml_parser/signals_xml_parser.h",
        "xml_parser/util_xml_parser.h",
        "coordinate_convert_tool.h",
        "proto_reorganization.h",
    ],
    deps = [
        "@glog//:glog",
        "//modules/common/math",
        "@proj4//:proj4",
        "@tinyxml2//:tinyxml2",
        "//modules/map/proto:map_proto",
        "//modules/common/status:status",
    ],
)

cpplint()
