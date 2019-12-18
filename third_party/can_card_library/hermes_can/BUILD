package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "hermes_can",
    srcs = select(
        {
            ":x86_mode": [
                "lib_x86_64/libbcan.so",
            ],
            ":arm_mode": [
                "lib_aarch64/libbcan.so",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = [
        "include/bcan.h",
    ],
    include_prefix = "hermes_can",
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
