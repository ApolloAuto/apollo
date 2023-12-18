package(
    default_visibility = ["//visibility:public"],
)

config_setting(
    name = "use_gpu",
    define_values = {
        "USE_GPU": "true",
    },
)

config_setting(
    name = "use_nvidia_gpu",
    define_values = {
        "USE_GPU": "true",
        "NVIDIA": "0",
        "AMD": "1",
        "GPU_PLATFORM": "NVIDIA",
    },
)

config_setting(
    name = "use_amd_gpu",
    define_values = {
        "USE_GPU": "true",
        "NVIDIA": "0",
        "AMD": "1",
        "GPU_PLATFORM": "AMD",
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
        "WITH_TELEOP": "true",
    },
)

config_setting(
    name = "enable_profiler",
    define_values = {
        "ENABLE_PROFILER": "true",
    },
)
