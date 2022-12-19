load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "v2x_testdata",
    srcs = glob(["fusion/test_data/*"]),
)

install(
    name = "testdata",
    data = [
        ":v2x_testdata"
    ],
    data_dest = "v2x/addition_data"
)


install(
    name = "install",
    data_dest = "v2x",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":v2x.BUILD",
    ],
    deps = [
        "//modules/v2x/fusion/apps:install",
        ":pb_hdrs",
        "//modules/v2x/v2x_proxy/app:install",
        "testdata",
        "//modules/v2x/proto:py_pb_v2x"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "v2x/include",
    data = [
        "//modules/v2x/proto:fusion_params_cc_proto",
        "//modules/v2x/proto:v2x_car_status_cc_proto",
        "//modules/v2x/proto:v2x_junction_cc_proto",
        "//modules/v2x/proto:v2x_monitor_cc_proto",
        "//modules/v2x/proto:v2x_obstacles_cc_proto",
        "//modules/v2x/proto:v2x_obu_rsi_cc_proto",
        "//modules/v2x/proto:v2x_obu_traffic_light_cc_proto",
        "//modules/v2x/proto:v2x_rsi_cc_proto",
        "//modules/v2x/proto:v2x_service_car_to_obu_cc_proto",
        "//modules/v2x/proto:v2x_service_obu_to_car_cc_proto",
        "//modules/v2x/proto:v2x_traffic_light_policy_cc_proto",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_v2x_src",
        ":install_v2x_hdrs"
    ],
)

install_src_files(
    name = "install_v2x_src",
    src_dir = ["."],
    dest = "v2x/src",
    filter = "*",
)

install_src_files(
    name = "install_v2x_hdrs",
    src_dir = ["."],
    dest = "v2x/include",
    filter = "*.h",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "dag/*.dag",
        "data/**",
        "launch/*.launch",
        "fusion/test_data/**",
    ]),
)
