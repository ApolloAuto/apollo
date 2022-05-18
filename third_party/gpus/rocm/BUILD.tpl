load(":build_defs.bzl", "rocm_header_library")
load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "using_hipcc",
    values = {
        "define": "using_rocm_hipcc=true",
    },
)

# Equivalent to using_clang && -c opt
config_setting(
    name = "using_clang_opt",
    values = {
        "define": "using_hip_clang=true",
        "compilation_mode": "opt",
    },
)

rocm_header_library(
    name = "rocm_headers",
    hdrs = [
        "rocm/rocm_config.h",
        ":rocm-include"
    ],
    include_prefix = "third_party/gpus",
    includes = [
        ".",
        "rocm/include",
    ],
)

rocm_header_library(
    name = "hipblas_headers",
    hdrs = [":hipblas-include"],
    include_prefix = "third_party/gpus/hipblas",
    strip_include_prefix = "hipblas/include",
    deps = [":rocm_headers"],
    includes = ["hipblas/include"],
)

rocm_header_library(
    name = "miopen_header",
    hdrs = [":miopen-include"],
    include_prefix = "third_party/gpus/miopen",
    strip_include_prefix = "miopen/include",
    deps = [":rocm_headers"],
    includes = ["miopen/include"],
)

cc_library(
    name = "rocm",
    visibility = ["//visibility:public"],
    deps = [
        ":rocm_headers",
        ":hip",
        ":rocblas",
        ":hipblas",
        ":miopen",
    ],
)

cc_library(
    name = "miopen",
    srcs = ["lib/libMIOpen.so"],
    data = ["lib/libMIOpen.so"],
    includes = [
        "include/",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hipblas",
    srcs = ["lib/libhipblas.so"],
    data = ["lib/libhipblas.so"],
    includes = [
        "include/",
    ],
)

cc_library(
    name = "migraphx",
    includes = [
        "./migraphx/include",
    ],
    linkopts = [
        "-L/opt/rocm/migraphx/lib",
        "-lmigraphx_gpu",
        "-lmigraphx_onnx", 
        "-lmigraphx_ref",
        "-lmigraphx_tf",
        "-lmigraphx_c",
        "-lmigraphx",
    ],
    linkstatic = 1,
)

cc_library(
    name = "rpp",
    includes = [
        "./rpp/include",
    ],
    linkopts = [
        "-L/opt/rocm/rpp/lib",
        "-lamd_rpp",
    ],
)

bzl_library(
    name = "build_defs_bzl",
    srcs = ["build_defs.bzl"],
    deps = [
        "@bazel_skylib//lib:selects",
    ],
)

%{copy_rules}
