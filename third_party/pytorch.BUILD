package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "pytorch",
    hdrs = glob(["*"]),
    srcs = [
        "lib/libtorch.so",
        "lib/libc10.so",
        "lib/libc10_cuda.so",
        "lib/libcaffe2.so",
        "lib/libcaffe2_detectron_ops_gpu.so",
        "lib/libcaffe2_gpu.so",
        "lib/libcaffe2_observers.so",
        "lib/libfoxi.so",
        "lib/libiomp5.so",
        "lib/libmklml_intel.so",
        "lib/libthnvrtc.so",
        "lib/libshm.so",
        "lib/libonnxifi.so",
    ],
    includes = [
        "include",
        "include/torch/csrc/api/include",
        "include/torch/csrc/api/include/torch"
    ],
    copts = [
        "-Iinclude",
        "-Iinclude/torch",
        "-Iinclude/torch/csrc/api/include/torch",
    ],
    linkopts = [
        "-Llib",
    ],
    deps = [
        "@gtest",
        "@python27",
    ]
)
