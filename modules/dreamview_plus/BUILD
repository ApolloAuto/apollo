load("//tools/platform:build_defs.bzl", "copts_if_teleop")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview_plus\\"']

filegroup(
    name = "frontend",
    srcs = glob(["frontend/dist/**/*"]),
)

apollo_cc_binary(
    name = "dreamview_plus",
    srcs = ["main.cc"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    data = [
        ":frontend",
    ],
    linkopts = [
        "-ltcmalloc",
        "-lprofiler",
    ],
    deps = [
        "//cyber",
        "//modules/dreamview_plus/backend:apollo_dreamview_plus_backend",
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

apollo_package()

cpplint()
