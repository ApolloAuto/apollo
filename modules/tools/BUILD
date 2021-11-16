load("//tools/install:install.bzl", "install", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        "//modules/tools/common:py_files",
        "//modules/tools/create_map:py_files",
        "//modules/tools/map_gen:py_files",
        "//modules/tools/record_play:py_files",
        "//modules/tools/sensor_calibration:runtime_files",
        "//modules/tools/vehicle_calibration:runtime_files",
    ],
    rename = {
        "modules/tools/sensor_calibration/extract_data.py": "extract_data",
        "modules/tools/record_play/rtk_recorder.py": "rtk_recorder",
        "modules/tools/record_play/rtk_player.py": "rtk_player",
        "modules/tools/vehicle_calibration/preprocess.py": "preprocess",
    },
    deps = [
        ":pb_tools",
        "//modules/tools/visualizer:install",
    ],
)

install_files(
    name = "pb_tools",
    dest = "modules/tools",
    files = [
        "//modules/tools/sensor_calibration/proto:extractor_config_py_pb2",
    ],
)
