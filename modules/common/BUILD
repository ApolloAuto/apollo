load("//tools/install:install.bzl", "install", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [
        ":pb_common",
        "//modules/common/data:install",
        "//modules/common/vehicle_model:install",
    ],
)

install_files(
    name = "pb_common",
    dest = "modules/common",
    files = [
        "//modules/common/proto:direction_py_pb2",
        "//modules/common/proto:drive_state_py_pb2",
        "//modules/common/proto:error_code_py_pb2",
        "//modules/common/proto:geometry_py_pb2",
        "//modules/common/proto:header_py_pb2",
        "//modules/common/proto:pnc_point_py_pb2",
        "//modules/common/proto:vehicle_signal_py_pb2",
        "//modules/common/proto:drive_event_py_pb2",
        "//modules/common/configs/proto:vehicle_config_py_pb2",
    ],
)
