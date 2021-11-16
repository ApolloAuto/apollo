load("//tools/install:install.bzl", "install", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [
        ":pb_map",
        "//modules/map/data:install",
        "//modules/map/relative_map:install",
        "//modules/map/tools:install",
    ],
)

install_files(
    name = "pb_map",
    dest = "modules/map",
    files = [
        "//modules/map/proto:map_clear_area_py_pb2",
        "//modules/map/proto:map_crosswalk_py_pb2",
        "//modules/map/proto:map_geometry_py_pb2",
        "//modules/map/proto:map_id_py_pb2",
        "//modules/map/proto:map_junction_py_pb2",
        "//modules/map/proto:map_lane_py_pb2",
        "//modules/map/proto:map_overlap_py_pb2",
        "//modules/map/proto:map_parking_space_py_pb2",
        "//modules/map/proto:map_pnc_junction_py_pb2",
        "//modules/map/proto:map_proto",
        "//modules/map/proto:map_py_pb2",
        "//modules/map/proto:map_road_py_pb2",
        "//modules/map/proto:map_rsu_py_pb2",
        "//modules/map/proto:map_signal_py_pb2",
        "//modules/map/proto:map_speed_bump_py_pb2",
        "//modules/map/proto:map_stop_sign_py_pb2",
        "//modules/map/proto:map_yield_sign_py_pb2",
        "//modules/map/relative_map/proto:navigation_py_pb2",
        "//modules/map/relative_map/proto:navigator_config_py_pb2",
        "//modules/map/relative_map/proto:relative_map_config_py_pb2",
    ],
)
