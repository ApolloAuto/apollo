load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

licenses(["restricted"])  # MPL2, portions GPL v3, LGPL v3, BSD-like

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

cc_library(
    name = "rocm_headers",
    hdrs = [
        "rocm/rocm_config.h",
        %{rocm_headers}
    ],
    includes = [
        ".",
        "rocm/include",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hip",
    srcs = ["rocm/lib/%{hip_lib}"],
    data = ["rocm/lib/%{hip_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "miopen",
    srcs = ["rocm/lib/%{miopen_lib}"],
    data = ["rocm/lib/%{miopen_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    deps = [
        ":rocblas",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_lib",
    srcs = ["rocm/lib/%{migraphx_lib}"],
    data = ["rocm/lib/%{migraphx_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_c_lib",
    srcs = ["rocm/lib/%{migraphx_c_lib}"],
    data = ["rocm/lib/%{migraphx_c_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_tf_lib",
    srcs = ["rocm/lib/%{migraphx_tf_lib}"],
    data = ["rocm/lib/%{migraphx_tf_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_device_lib",
    srcs = ["rocm/lib/%{migraphx_device_lib}"],
    data = ["rocm/lib/%{migraphx_device_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_gpu_lib",
    srcs = ["rocm/lib/%{migraphx_gpu_lib}"],
    data = ["rocm/lib/%{migraphx_gpu_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_ref_lib",
    srcs = ["rocm/lib/%{migraphx_ref_lib}"],
    data = ["rocm/lib/%{migraphx_ref_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx_lib_onnx",
    srcs = ["rocm/lib/%{migraphx_onnx_lib}"],
    data = ["rocm/lib/%{migraphx_onnx_lib}"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "migraphx",
    visibility = ["//visibility:public"],
    deps = [
        ":migraphx_lib",
        ":migraphx_c_lib",
        ":migraphx_tf_lib",
        ":migraphx_device_lib",
        ":migraphx_gpu_lib",
        ":migraphx_ref_lib",
        ":migraphx_lib_onnx",
        ":miopen",
    ],
)

cc_library(
    name = "rocm",
    visibility = ["//visibility:public"],
    deps = [
        ":rocm_headers",
        ":hip",
        ":hipblas",
        ":miopen",
        ":migraphx",
    ],
)

cc_library(
    name = "hipblas",
    srcs = ["rocm/lib/%{hipblas_lib}"],
    data = ["rocm/lib/%{hipblas_lib}"],
)

cc_library(
    name = "rocblas",
    srcs = ["rocm/lib/%{rocblas_lib}"],
    data = ["rocm/lib/%{rocblas_lib}"],
)

cc_library(
    name = "rpp",
    includes = [
        ".",
        "rocm/include",
    ],
    linkopts = [
        "-L/opt/rocm/rpp/lib",
        "-lamd_rpp",
    ],
    defines = [
        "GPU_SUPPORT",
        "RPP_BACKEND_HIP",
    ],
)

bzl_library(
    name = "build_defs_bzl",
    srcs = ["build_defs.bzl"],
    deps = [
        "@bazel_skylib//lib:selects",
    ],
)

filegroup(
    name = "rocm_root",
    srcs = [
        "rocm/bin/clang-offload-bundler",
    ],
)

py_library(
    name = "rocm_config_py",
    srcs = ["rocm/rocm_config.py"]
)

%{copy_rules}
