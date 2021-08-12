load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/platform:build_defs.bzl", "copts_if_teleop")
load("//tools/install:install.bzl", "install", "install_files")
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
    data = [
        ":dreamview_conf",
        ":frontend",
    ],
    targets = ["dreamview"],
    deps = [
        ":pb_dreamview",
        "//cyber:install",
    ],
)

install_files(
    name = "pb_dreamview",
    dest = "modules/dreamview",
    files = [
        "//modules/dreamview/proto:chart_py_pb2",
        "//modules/dreamview/proto:preprocess_table_py_pb2",
    ],
)

cpplint()
