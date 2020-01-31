package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "pytorch_gpu",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["include/torch/csrc/api/include/**/*.h"]),
    includes = [
        "include",
        "include/torch/csrc/api/include",
    ],
    linkopts = ["-lpython3.6m"],
)
