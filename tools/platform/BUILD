package(
    default_visibility = ["//visibility:public"],
)

config_setting(
    name = "aarch64_mode",
    values = {"cpu": "aarch64"},
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "use_gpu",
    values = {
        "define": "USE_GPU=true",
    },
)

config_setting(
    name = "use_esd_can",
    values = {
        "define": "CAN_CARD=esd_can",
    },
)
