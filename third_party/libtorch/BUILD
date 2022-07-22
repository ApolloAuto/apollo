package(
    default_visibility = ["//visibility:public"],
)
cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_gpu": [
            "@libtorch_gpu",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)