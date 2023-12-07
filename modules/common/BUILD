# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

# install(
#     name = "install",
#     library_dest = "common/lib",
#     data_dest = "common",
#     data = [
#         ":cyberfile.xml",
#         "common.BUILD"
#     ],
#     deps = [
#         ":pb_common",
#         ":pb_hdrs",
#         "//modules/common/data:install",
#         "//modules/common/vehicle_model:install",
#         "//modules/common/adapters:install",
#         "//modules/common/configs:install",
#         "//modules/common/filters:install",
#         "//modules/common/kv_db:install",
#         "//modules/common/latency_recorder:install",
#         "//modules/common/math:install",
# 	    "//modules/common/status:install",
#         "//modules/common/monitor_log:install",
#         "//modules/common/util:install",
#         "//modules/common/vehicle_state:install",
#         "//modules/common/vehicle_state/proto:py_pb_common_vehicle_state",
#         "//modules/common/vehicle_model/proto:py_pb_common_vehicle_model",
#         "//modules/common/util/testdata:py_pb_common_util",
#         "//modules/common/latency_recorder/proto:py_pb_common_latency_record",
#         "//modules/common/adapters/proto:py_pb_common_adapter"
#     ],
# )

# todo:// keep *.pb.h only
# install(
#     name = "pb_hdrs",
#     data_dest = "common/include",
#     data = [
#         "//modules/common/adapters/proto:adapter_config_cc_proto",
#         "//modules/common/latency_recorder/proto:latency_record_cc_proto",
#         "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
#         "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
#         "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_common",
#     dest = "common",
#     files = [
#         "//modules/common_msgs/basic_msgs:direction_py_pb2",
#         "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
#         "//modules/common_msgs/basic_msgs:error_code_py_pb2",
#         "//modules/common_msgs/basic_msgs:geometry_py_pb2",
#         "//modules/common_msgs/basic_msgs:header_py_pb2",
#         "//modules/common_msgs/basic_msgs:pnc_point_py_pb2",
#         "//modules/common_msgs/basic_msgs:vehicle_signal_py_pb2",
#         "//modules/common_msgs/basic_msgs:drive_event_py_pb2",
#     ],
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_common_src",
#         ":install_common_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_common_src",
#     src_dir = ["."],
#     dest = "common/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_common_hdrs",
#     src_dir = ["."],
#     dest = "common/include",
#     filter = "*.h",
# )

apollo_package()
