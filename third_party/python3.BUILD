package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "python3",
    srcs = select(
        {
            ":x86_mode": glob([
                "lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6.so",
            ]),
            ":arm_mode": glob([
                "lib/python3.6/config-3.6m-aarch64-linux-gnu/libpython3.6.so",
            ]),
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = glob([
        "include/python3.6/*.h",
    ]),
    includes = ["include/python3.6"],
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
