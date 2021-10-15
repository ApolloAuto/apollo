load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    deps = [
        ":pb_localization",
        "//modules/localization/msf:install",
        "//modules/localization/ndt:install",
        "//modules/localization/rtk:install",
    ],
)

install_files(
    name = "pb_localization",
    dest = "modules/localization",
    files = [
        "//modules/localization/proto:gnss_pnt_result_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/localization/proto:imu_py_pb2",
        "//modules/localization/proto:localization_config_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/localization/proto:localization_status_py_pb2",
        "//modules/localization/proto:measure_py_pb2",
        "//modules/localization/proto:pose_py_pb2",
        "//modules/localization/proto:rtk_config_py_pb2",
        "//modules/localization/proto:sins_pva_py_pb2",
    ],
)

cpplint()
