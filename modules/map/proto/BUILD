load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "map_proto",
    deps = [
        ":map_proto_lib",
    ],
)

proto_library(
    name = "map_proto_lib",
    srcs = [
        "map.proto",
        "map_clear_area.proto",
        "map_crosswalk.proto",
        "map_geometry.proto",
        "map_id.proto",
        "map_junction.proto",
        "map_lane.proto",
        "map_overlap.proto",
        "map_parking.proto",
        "map_road.proto",
        "map_signal.proto",
        "map_speed_bump.proto",
        "map_speed_control.proto",
        "map_stop_sign.proto",
        "map_yield_sign.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)

cpplint()
