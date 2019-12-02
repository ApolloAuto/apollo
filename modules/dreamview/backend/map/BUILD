load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_service",
    srcs = ["map_service.cc"],
    hdrs = ["map_service.h"],
    deps = [
        "//modules/common/util:json_util",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//third_party/json",
        "@boost",
        "@glog",
    ],
)

cc_test(
    name = "map_service_test",
    size = "small",
    srcs = ["map_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":map_service",
        "@gtest//:main",
    ],
)

cpplint()
