load("//tools/platform:build_defs.bzl", "copts_if_teleop")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

filegroup(
    name = "frontend",
    srcs = glob(["frontend/dist/**/*"]),
)

apollo_cc_binary(
    name = "dreamview",
    srcs = ["main.cc"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    data = [
        ":frontend",
    ],
    deps = [
        "//cyber",
        "//modules/dreamview/backend:apollo_dreamview_backend",
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
