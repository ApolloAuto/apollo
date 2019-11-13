package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "python27",
    srcs = select(
        {
            ":x86_mode": glob([
                "lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so",
            ]),
            ":arm_mode": glob([
                "lib/python2.7/config-aarch64-linux-gnu/libpython2.7.so",
            ]),
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = glob([
        "include/python2.7/*.h",
    ]),
    includes = ["include/python2.7"],
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
