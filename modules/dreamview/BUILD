load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/platform:build_defs.bzl", "copts_if_teleop")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

filegroup(
    name = "frontend",
    srcs = glob(["frontend/dist/**/*"]),
)

cc_binary(
    name = "dreamview",
    srcs = ["main.cc"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    data = [
        ":frontend",
    ],
    deps = [
        "//modules/dreamview/backend",
    ],
)

filegroup(
    name = "dreamview_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
        "conf/hmi_modes/*.txt",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    library_dest = "dreamview/lib",
    data_dest = "dreamview",
    runtime_dest = "dreamview/bin",
    data = [
        ":dreamview_conf",
        ":frontend",
        "cyberfile.xml",
        "dreamview.BUILD"
    ],
    targets = ["dreamview"],
    deps = [
        ":pb_dreamview",
        ":pb_hdrs",
        "//modules/dreamview/backend/common:install",
        "//modules/dreamview/backend/hmi:install",
        "//modules/dreamview/backend/map:install",
        "//modules/dreamview/backend:install",
        "//modules/dreamview/backend/sim_control_manager/common:install",
        "//modules/dreamview/proto:py_pb_dreamview"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "dreamview/include",
    data = [
        "//modules/dreamview/proto:camera_update_cc_proto",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/dreamview/proto:data_collection_table_cc_proto",
        "//modules/dreamview/proto:point_cloud_cc_proto",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:dv_plugin_msg_cc_proto",
        "//modules/dreamview/proto:plugin_config_cc_proto",
        "//modules/dreamview/backend/teleop/proto:daemon_cmd_cc_proto",
        "//modules/dreamview/backend/teleop/proto:daemon_rpt_cc_proto",
        "//modules/dreamview/backend/teleop/proto:modem_info_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:dynamic_model_conf_cc_proto",
    ],
)

install_files(
    name = "pb_dreamview",
    dest = "dreamview",
    files = [
        "//modules/common_msgs/dreamview_msgs:chart_py_pb2",
        "//modules/dreamview/proto:preprocess_table_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_dreamview_src",
        ":install_dreamview_hdrs"
    ],
)

install_src_files(
    name = "install_dreamview_src",
    src_dir = ["."],
    dest = "dreamview/src",
    filter = "*",
)

install_src_files(
    name = "install_dreamview_hdrs",
    src_dir = ["."],
    dest = "dreamview/include",
    filter = "*.h",
)

cpplint()
