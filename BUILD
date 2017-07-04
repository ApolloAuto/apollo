package(
    default_visibility = ["//visibility:public"],
)

config_setting(
    name = "arm64",
    values = {
        "define": "ARCH=arm64",
    },
)

config_setting(
    name = "x86_64",
    values = {
        "define": "ARCH=x86_64",
    },
)

exports_files([
    "CPPLINT.cfg",
])
