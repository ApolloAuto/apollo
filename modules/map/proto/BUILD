load("//tools:py_proto.bzl", "py_proto")

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
        "map_parking_space.proto",
        "map_pnc_junction.proto",
        "map_road.proto",
        "map_signal.proto",
        "map_speed_bump.proto",
        "map_speed_control.proto",
        "map_stop_sign.proto",
        "map_yield_sign.proto",
    ],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
    ],
)

py_proto(
    name = "map_yield_sign_pb2",
    src = "map_yield_sign.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_road_pb2",
    src = "map_road.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_stop_sign_pb2",
    src = "map_stop_sign.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_geometry_pb2",
    src = "map_geometry.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
    ],
)

py_proto(
    name = "map_signal_pb2",
    src = "map_signal.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_overlap_pb2",
    src = "map_overlap.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_clear_area_pb2",
    src = "map_clear_area.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_id_pb2",
    src = "map_id.proto",
)

py_proto(
    name = "map_speed_control_pb2",
    src = "map_speed_control.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
    ],
)

py_proto(
    name = "map_lane_pb2",
    src = "map_lane.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_junction_pb2",
    src = "map_junction.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_parking_space_pb2",
    src = "map_parking_space.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_speed_bump_pb2",
    src = "map_speed_bump.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_crosswalk_pb2",
    src = "map_crosswalk.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)

py_proto(
    name = "map_pb2",
    src = "map.proto",
    deps = [
        "//modules/map/proto:map_clear_area_pb2",
        "//modules/map/proto:map_crosswalk_pb2",
        "//modules/map/proto:map_junction_pb2",
        "//modules/map/proto:map_lane_pb2",
        "//modules/map/proto:map_overlap_pb2",
        "//modules/map/proto:map_parking_space_pb2",
        "//modules/map/proto:map_pnc_junction_pb2",
        "//modules/map/proto:map_road_pb2",
        "//modules/map/proto:map_signal_pb2",
        "//modules/map/proto:map_speed_bump_pb2",
        "//modules/map/proto:map_stop_sign_pb2",
        "//modules/map/proto:map_yield_sign_pb2",
    ],
)

py_proto(
    name = "map_pnc_junction_pb2",
    src = "map_pnc_junction.proto",
    deps = [
        "//modules/map/proto:map_geometry_pb2",
        "//modules/map/proto:map_id_pb2",
    ],
)
