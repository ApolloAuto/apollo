load("//tools:cpplint.bzl", "cpplint")

# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

# install(
#     name = "testdata",
#     data = [
#         ":localization_testdata"
#     ],
#     data_dest = "localization/addition_data"
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

# install(
#     name = "install",
#     data_dest = "localization",
#     data = [
#         ":runtime_data",
#         "cyberfile.xml",
#         "localization.BUILD",
#     ],
#     deps = [
#         ":pb_localization",
#         ":pb_hdrs",
#         "//modules/localization/msf:install",
#         "//modules/localization/ndt:install",
#         "//modules/localization/rtk:install",
#         "//modules/localization/msf/local_tool/data_extraction:install",
#         "//modules/localization/msf/local_tool/local_visualization/offline_visual:install",
#         "//modules/localization/msf/local_tool/local_visualization/online_visual:install",
#         "//modules/localization/msf/local_tool/map_creation:install",
#         "//modules/localization/ndt/map_creation:install",
#         "//modules/localization/common:install",
#         ":testdata",
#         "//modules/localization/proto:py_pb_localization"
#     ],
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "localization/include",
#     data = [
#         "//modules/localization/proto:gnss_pnt_result_cc_proto",
#         "//modules/localization/proto:localization_config_cc_proto",
#         "//modules/localization/proto:measure_cc_proto",
#         "//modules/localization/proto:rtk_config_cc_proto",
#         "//modules/localization/proto:sins_pva_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_localization",
#     dest = "localization",
#     files = [
#         "//modules/localization/proto:gnss_pnt_result_py_pb2",
#         "//modules/common_msgs/localization_msgs:gps_py_pb2",
#         "//modules/common_msgs/localization_msgs:imu_py_pb2",
#         "//modules/localization/proto:localization_config_py_pb2",
#         "//modules/common_msgs/localization_msgs:localization_py_pb2",
#         "//modules/common_msgs/localization_msgs:localization_status_py_pb2",
#         "//modules/localization/proto:measure_py_pb2",
#         "//modules/common_msgs/localization_msgs:pose_py_pb2",
#         "//modules/localization/proto:rtk_config_py_pb2",
#         "//modules/localization/proto:sins_pva_py_pb2",
#     ],
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_localization_src",
#         ":install_localization_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_localization_src",
#     src_dir = ["."],
#     dest = "localization/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_localization_hdrs",
#     src_dir = ["."],
#     dest = "localization/include",
#     filter = "*.h",
# )

apollo_package()

cpplint()
