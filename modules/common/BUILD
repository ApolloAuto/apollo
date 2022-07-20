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
        "//modules/common_msgs/basic_msgs:direction_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
        "//modules/common_msgs/basic_msgs:error_code_py_pb2",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
        "//modules/common_msgs/basic_msgs:pnc_point_py_pb2",
        "//modules/common_msgs/basic_msgs:vehicle_signal_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_event_py_pb2",
        "//modules/common/configs/proto:vehicle_config_py_pb2",
    ],
)
