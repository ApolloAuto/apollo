load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libwato_component.so",
    linkopts = [
        "-lswscale",
        "-lavformat",
        "-lavcodec",
        "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":compress_component_lib",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = ['-DMODULE_NAME=\\"wato\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/wato/proto:wato_proto",
        "//modules/drivers/proto:sensor_proto",
        "@opencv",
    ],
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)

cpplint()
# ./apollo.sh build_opt_gpu
