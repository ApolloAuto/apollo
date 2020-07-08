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
    define_values = {
        "USE_GPU": "1",
    },
)

config_setting(
    name = "use_esd_can",
    define_values = {
        "USE_ESD_CAN": "true",
    },
)

config_setting(
    name = "with_teleop",
    define_values = {
        "WITH_TELEOP": "1",
    },
)
