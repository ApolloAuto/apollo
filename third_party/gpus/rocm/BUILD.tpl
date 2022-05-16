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

# TODO(emankov): Uncomment rocm/rocm_config.h after setting it up in rocm_configure.bzl
cc_library(
    name = "rocm_headers",
    hdrs = [
#       "rocm/rocm_config.h",
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
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "rocm",
    visibility = ["//visibility:public"],
    deps = [
        ":rocm_headers",
        ":hip",
        ":hipblas",
        ":miopen",
    ],
)

bzl_library(
    name = "build_defs_bzl",
    srcs = ["build_defs.bzl"],
)

cc_library(
    name = "hipblas",
    srcs = ["rocm/lib/%{hipblas_lib}"],
    data = ["rocm/lib/%{hipblas_lib}"],
)

filegroup(
    name = "rocm_root",
    srcs = [
        "rocm/bin/clang-offload-bundler",
    ],
)

%{copy_rules}
