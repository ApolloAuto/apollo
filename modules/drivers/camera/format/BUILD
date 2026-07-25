load("//tools:cpplint.bzl", "cpplint")

load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library", "if_cuda")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# yuv2rgb.cu can only be compiled with the CUDA toolchain. Since this
# package's generated `:install` target is unconditionally pulled in by
# //modules/drivers/camera's own `:install` aggregation (via
# apollo_package()'s automatic subpackage deps), simply excluding this
# package from CPU builds isn't enough to keep it from being built. Instead,
# drop the CUDA source when CUDA isn't configured so `:convert` degrades to
# an empty (but buildable) library on CPU-only builds.
cuda_library(
    name = "convert",
    srcs = if_cuda(["yuv2rgb.cu"], []),
    hdrs = ["yuv2rgb.h"],
    deps = [
        "//cyber",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cuda_headers",
    ]
)

apollo_package(enable_source=False)
cpplint()
