# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

# install(
#     name = "install",
#     data_dest = "tools",
#     data = [
#         "//modules/tools/common:py_files",
#         "//modules/tools/create_map:py_files",
#         "//modules/tools/map_gen:py_files",
#         "//modules/tools/record_play:py_files",
#         "//modules/tools/sensor_calibration:runtime_files",
#         "//modules/tools/vehicle_calibration:runtime_files",
#         ":cyberfile.xml",
#         ":tools.BUILD",
#     ],
#     rename = {
#         "modules/tools/sensor_calibration/extract_data.py": "extract_data",
#         "modules/tools/record_play/rtk_recorder.py": "rtk_recorder",
#         "modules/tools/record_play/rtk_player.py": "rtk_player",
#         "modules/tools/vehicle_calibration/preprocess.py": "preprocess",
#     },
#     deps = [
#         ":pb_tools",
#         ":pb_hdrs",
#         "//modules/tools/visualizer:install",
#         "//modules/tools/manual_traffic_light:install",
#         "//modules/tools/prediction/fake_prediction:install",
#     ],
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "tools/include",
#     data = [
#         "//modules/tools/navigator/dbmap/proto:dbmap_cc_proto",
#         "//modules/tools/prediction/data_pipelines/proto:cruise_model_cc_proto",
#         "//modules/tools/prediction/data_pipelines/proto:fnn_model_cc_proto",
#         "//modules/tools/sensor_calibration/proto:extractor_config_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_tools",
#     dest = "tools",
#     files = [
#         "//modules/tools/sensor_calibration/proto:extractor_config_py_pb2",
#     ],
# )

# install(
#     name = "install",
#     deps = [
#         "//modules/tools/control_info:install",
#         "//modules/tools/create_map:install",
#         "//modules/tools/data_cleaner:install",
#         "//modules/tools/dump_gpsbin:install",
#         "//modules/tools/gen_vehicle_protocol:install",
#         "//modules/tools/localization:install",
#         "//modules/tools/manual_traffic_light:install",
#         "//modules/tools/map_datachecker:install",
#         "//modules/tools/map_gen:install",
#         "//modules/tools/mapshow:install",
#         "//modules/tools/mapviewers:install",
#         "//modules/tools/mock_routing:install",
#         "//modules/tools/navigation:install",
#         "//modules/tools/open_space_visualization:install",
#         "//modules/tools/perception:install",
#         "//modules/tools/planning/plot_trajectory:install",
#         "//modules/tools/plot_control:install",
#         "//modules/tools/plot_planning:install",
#         "//modules/tools/plot_trace:install",
#         "//modules/tools/realtime_plot:install",
#         "//modules/tools/record_analyzer:install",
#         "//modules/tools/record_parse_save:install",
#         "//modules/tools/record_play:install",
#         "//modules/tools/replay:install",
#         "//modules/tools/restore_video_record:install",
#         "//modules/tools/rosbag:install",
#         "//modules/tools/routing:install",
#         "//modules/tools/sensor_calibration:install",
#         "//modules/tools/vehicle_calibration:install",
#         "//modules/tools/vehicle_profile:install",
#         "//modules/tools/visualizer:install",
#         ":pb_hdrs"
#     ],
#     data = [
#         ":cyberfile.xml",
#         ":tools.BUILD"
#     ],
#     data_dest = "tools",
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_tools_src",
#         ":install_tools_py"
#     ],
# )

# install_src_files(
#     name = "install_tools_src",
#     src_dir = ["."],
#     dest = "tools/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_tools_py",
#     src_dir = ["."],
#     dest = "tools/python/modules/tools",
#     filter = "*.py",
# )

apollo_package()
