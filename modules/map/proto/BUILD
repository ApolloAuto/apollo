load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "map_proto",
    protos = [
        "map.proto",
        "map_clear_area.proto",
        "map_crosswalk.proto",
        "map_geometry.proto",
        "map_id.proto",
        "map_junction.proto",
        "map_lane.proto",
        "map_overlap.proto",
        "map_parking_space.proto",
        "map_signal.proto",
        "map_speed_bump.proto",
        "map_stop_sign.proto",
        "map_yield_sign.proto",
        "routing.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "map_proto_pylib",
    protos = [
        "map.proto",
        "map_clear_area.proto",
        "map_crosswalk.proto",
        "map_geometry.proto",
        "map_id.proto",
        "map_junction.proto",
        "map_lane.proto",
        "map_overlap.proto",
        "map_parking_space.proto",
        "map_signal.proto",
        "map_speed_bump.proto",
        "map_stop_sign.proto",
        "map_yield_sign.proto",
        "routing.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)

cpplint()
