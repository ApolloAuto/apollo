package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "cuda",
    includes = ["include"],
    linkopts = select(
        {
            ":x86_mode": [
                "-L/usr/lib/x86_64-linux-gnu/",
            ],
            ":arm_mode": [
                "-L/usr/lib/aarch64-linux-gnu/",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ) + [
        "-lgomp",
        "-L/usr/local/cuda/lib64",
        "-lOpenCL",
        "-lcublas",
        "-lcudart",
        "-lcudnn",
        "-lcufft",
        "-lcufftw",
        "-lcuinj64",
        "-lcurand",
        "-lcusolver",
        "-lcusparse",
        "-lnppc",
        "-lnppial",
        "-lnppicc",
        "-lnppicom",
        "-lnppidei",
        "-lnppif",
        "-lnppig",
        "-lnppim",
        "-lnppist",
        "-lnppisu",
        "-lnppitc",
        "-lnpps",
        "-lnvToolsExt",
        "-lnvblas",
        "-lnvgraph",
        "-lnvrtc-builtins",
        "-lnvrtc",
        "-L/usr/local/cuda/lib64/stubs",
        "-lcublas",
        "-lcuda",
        "-lcufft",
        "-lcufftw",
        "-lcurand",
        "-lcusolver",
        "-lcusparse",
        "-lnppc",
        "-lnppial",
        "-lnppicc",
        "-lnppicom",
        "-lnppidei",
        "-lnppif",
        "-lnppig",
        "-lnppim",
        "-lnppist",
        "-lnppisu",
        "-lnppitc",
        "-lnpps",
        "-lnvgraph",
        "-lnvidia-ml",
        "-lnvrtc",
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
