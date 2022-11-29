load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "map/lib",
    targets = [":libopendrive_adapter.so"],
    visibility = ["//visibility:public"],
)
cc_binary(
    name = "libopendrive_adapter.so",
    srcs = [
        "opendrive_adapter.cc",
        "opendrive_adapter.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":proto_organizer",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/map/hdmap/adapter/xml_parser",
    ],
)

cc_library(
    name = "opendrive_adapter",
    srcs = ["libopendrive_adapter.so"],
    hdrs = [
        "opendrive_adapter.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/hdmap/adapter/xml_parser",
        "@com_google_absl//:absl",
        "@proj",
        "@tinyxml2",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "proto_organizer",
    srcs = ["proto_organizer.cc"],
    hdrs = ["proto_organizer.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/map/hdmap/adapter/xml_parser",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "@com_google_absl//:absl",
    ],
    visibility = ["//visibility:private"],
    linkstatic = True,
)

cpplint()
