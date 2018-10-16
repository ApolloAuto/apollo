load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_proto_library(
    name = "multicue_proto",
    deps = [
        ":multicue_proto_lib",
    ],
)

proto_library(
    name = "multicue_proto_lib",
    srcs = [
        "multicue.proto",
    ],
)


cc_library(
    name = "obj_mapper",
    srcs = [
        "obj_mapper.cc",
    ],
    hdrs = [ 
        "obj_mapper.h",
    ],
    deps = [
        "//cybertron",
        "//modules/perception/camera/common:common",
    ],
)

cc_library(
    name = "multicue_obstacle_transformer",
    srcs = [
        "multicue_obstacle_transformer.cc",
    ],
    hdrs = [ 
        "multicue_obstacle_transformer.h",
    ],
    deps = [
        ":multicue_proto",
        ":obj_mapper",
        "//cybertron",
        "//modules/common/util:file_util",
        "//modules/perception/camera/common:common",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/singleton:singleton",
    ],
)

cpplint()
