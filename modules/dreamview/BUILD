load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/platform:build_defs.bzl", "copts_if_teleop")
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

cpplint()
