load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_proto_library(
    name = "location_refiner_proto",
    deps = [
        ":location_refiner_proto_lib",
    ],
)

proto_library(
    name = "location_refiner_proto_lib",
    srcs = [
        "location_refiner.proto",
    ],
)


cc_library(
    name = "location_refiner_obstacle_postprocessor",
    srcs = [
        "location_refiner_obstacle_postprocessor.cc",
    ],
    hdrs = [ 
        "location_refiner_obstacle_postprocessor.h",
    ],
    deps = [
        ":obj_postprocessor",
        ":location_refiner_proto",
        "//cyber",
        "//modules/common/util:file_util",
        "//modules/perception/camera/common:common",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/singleton:singleton",
    ],
)

cc_library(
    name = "obj_postprocessor",
    srcs = [
        "obj_postprocessor.cc",
    ],
    hdrs = [ 
        "obj_postprocessor.h",
    ],
    deps = [
        "//modules/perception/camera/common:common",
    ],
)

cpplint()
