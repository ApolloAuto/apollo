load("@bazel_skylib//:bzl_library.bzl", "bzl_library")


package(default_visibility = ["//visibility:public"])


config_setting(
    name = "using_rocm",
    values = {
        "define": "using_rocm=true",
    },
)

cc_library(
    name = "rocm_headers",
    hdrs = [
        ":rocm-include",

    ],
    includes = [
        ".",
        "rocm/include",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hip",
    srcs = ["rocm/lib/libamdhip64.so"],
    data = ["rocm/lib/libamdhip64.so"],
    includes = [
        ".",
        "rocm/include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

bzl_library(
    name = "build_defs_bzl",
    srcs = ["build_defs.bzl"],
)




filegroup(
    name = "rocm_root",
    srcs = [
        "rocm/bin/clang-offload-bundler",
    ],
)

genrule(
    name = "rocm-include",
    outs = [
        "rocm/include/hip/amd_detail/amd_channel_descriptor.h",
        "rocm/include/hip/amd_detail/amd_device_functions.h",
        "rocm/include/hip/amd_detail/amd_hip_atomic.h",
        "rocm/include/hip/amd_detail/amd_hip_common.h",
        "rocm/include/hip/amd_detail/amd_hip_complex.h",
        "rocm/include/hip/amd_detail/amd_hip_cooperative_groups.h",
        "rocm/include/hip/amd_detail/amd_hip_fp16.h",
        "rocm/include/hip/amd_detail/amd_hip_runtime.h",
        "rocm/include/hip/amd_detail/amd_hip_unsafe_atomics.h",
        "rocm/include/hip/amd_detail/amd_hip_vector_types.h",
        "rocm/include/hip/amd_detail/amd_math_functions.h",
        "rocm/include/hip/amd_detail/amd_surface_functions.h",
        "rocm/include/hip/amd_detail/concepts.hpp",
        "rocm/include/hip/amd_detail/device_library_decls.h",
        "rocm/include/hip/amd_detail/functional_grid_launch.hpp",
        "rocm/include/hip/amd_detail/grid_launch.h",
        "rocm/include/hip/amd_detail/grid_launch.hpp",
        "rocm/include/hip/amd_detail/grid_launch_GGL.hpp",
        "rocm/include/hip/amd_detail/helpers.hpp",
        "rocm/include/hip/amd_detail/hip_cooperative_groups_helper.h",
        "rocm/include/hip/amd_detail/hip_fp16_gcc.h",
        "rocm/include/hip/amd_detail/hip_fp16_math_fwd.h",
        "rocm/include/hip/amd_detail/hip_ldg.h",
        "rocm/include/hip/amd_detail/hip_memory.h",
        "rocm/include/hip/amd_detail/hip_prof_str.h",
        "rocm/include/hip/amd_detail/hip_runtime_prof.h",
        "rocm/include/hip/amd_detail/host_defines.h",
        "rocm/include/hip/amd_detail/hsa_helpers.hpp",
        "rocm/include/hip/amd_detail/llvm_intrinsics.h",
        "rocm/include/hip/amd_detail/macro_based_grid_launch.hpp",
        "rocm/include/hip/amd_detail/math_fwd.h",
        "rocm/include/hip/amd_detail/ockl_image.h",
        "rocm/include/hip/amd_detail/program_state.hpp",
        "rocm/include/hip/amd_detail/texture_fetch_functions.h",
        "rocm/include/hip/amd_detail/texture_indirect_functions.h",
        "rocm/include/hip/channel_descriptor.h",
        "rocm/include/hip/device_functions.h",
        "rocm/include/hip/driver_types.h",
        "rocm/include/hip/hcc_detail/amd_channel_descriptor.h",
        "rocm/include/hip/hcc_detail/amd_device_functions.h",
        "rocm/include/hip/hcc_detail/amd_hip_atomic.h",
        "rocm/include/hip/hcc_detail/amd_hip_common.h",
        "rocm/include/hip/hcc_detail/amd_hip_complex.h",
        "rocm/include/hip/hcc_detail/amd_hip_cooperative_groups.h",
        "rocm/include/hip/hcc_detail/amd_hip_fp16.h",
        "rocm/include/hip/hcc_detail/amd_hip_runtime.h",
        "rocm/include/hip/hcc_detail/amd_hip_unsafe_atomics.h",
        "rocm/include/hip/hcc_detail/amd_hip_vector_types.h",
        "rocm/include/hip/hcc_detail/amd_math_functions.h",
        "rocm/include/hip/hcc_detail/amd_surface_functions.h",
        "rocm/include/hip/hcc_detail/concepts.hpp",
        "rocm/include/hip/hcc_detail/device_library_decls.h",
        "rocm/include/hip/hcc_detail/functional_grid_launch.hpp",
        "rocm/include/hip/hcc_detail/grid_launch.h",
        "rocm/include/hip/hcc_detail/grid_launch.hpp",
        "rocm/include/hip/hcc_detail/grid_launch_GGL.hpp",
        "rocm/include/hip/hcc_detail/helpers.hpp",
        "rocm/include/hip/hcc_detail/hip_cooperative_groups_helper.h",
        "rocm/include/hip/hcc_detail/hip_fp16_gcc.h",
        "rocm/include/hip/hcc_detail/hip_fp16_math_fwd.h",
        "rocm/include/hip/hcc_detail/hip_ldg.h",
        "rocm/include/hip/hcc_detail/hip_memory.h",
        "rocm/include/hip/hcc_detail/hip_prof_str.h",
        "rocm/include/hip/hcc_detail/hip_runtime_prof.h",
        "rocm/include/hip/hcc_detail/host_defines.h",
        "rocm/include/hip/hcc_detail/hsa_helpers.hpp",
        "rocm/include/hip/hcc_detail/llvm_intrinsics.h",
        "rocm/include/hip/hcc_detail/macro_based_grid_launch.hpp",
        "rocm/include/hip/hcc_detail/math_fwd.h",
        "rocm/include/hip/hcc_detail/ockl_image.h",
        "rocm/include/hip/hcc_detail/program_state.hpp",
        "rocm/include/hip/hcc_detail/texture_fetch_functions.h",
        "rocm/include/hip/hcc_detail/texture_indirect_functions.h",
        "rocm/include/hip/hip_bfloat16.h",
        "rocm/include/hip/hip_common.h",
        "rocm/include/hip/hip_complex.h",
        "rocm/include/hip/hip_cooperative_groups.h",
        "rocm/include/hip/hip_ext.h",
        "rocm/include/hip/hip_fp16.h",
        "rocm/include/hip/hip_hcc.h",
        "rocm/include/hip/hip_profile.h",
        "rocm/include/hip/hip_runtime.h",
        "rocm/include/hip/hip_runtime_api.h",
        "rocm/include/hip/hip_texture_types.h",
        "rocm/include/hip/hip_vector_types.h",
        "rocm/include/hip/hip_version.h",
        "rocm/include/hip/hiprtc.h",
        "rocm/include/hip/library_types.h",
        "rocm/include/hip/math_functions.h",
        "rocm/include/hip/nvcc_detail/nvidia_channel_descriptor.h",
        "rocm/include/hip/nvcc_detail/nvidia_hip_complex.h",
        "rocm/include/hip/nvcc_detail/nvidia_hip_cooperative_groups.h",
        "rocm/include/hip/nvcc_detail/nvidia_hip_runtime.h",
        "rocm/include/hip/nvcc_detail/nvidia_hip_runtime_api.h",
        "rocm/include/hip/nvcc_detail/nvidia_hip_texture_types.h",
        "rocm/include/hip/nvcc_detail/nvidia_hiprtc.h",
        "rocm/include/hip/nvidia_detail/nvidia_channel_descriptor.h",
        "rocm/include/hip/nvidia_detail/nvidia_hip_complex.h",
        "rocm/include/hip/nvidia_detail/nvidia_hip_cooperative_groups.h",
        "rocm/include/hip/nvidia_detail/nvidia_hip_runtime.h",
        "rocm/include/hip/nvidia_detail/nvidia_hip_runtime_api.h",
        "rocm/include/hip/nvidia_detail/nvidia_hip_texture_types.h",
        "rocm/include/hip/nvidia_detail/nvidia_hiprtc.h",
        "rocm/include/hip/surface_types.h",
        "rocm/include/hip/texture_types.h",
    ],
    cmd = """cp -rLf "/opt/rocm/hip/include/." "$(@D)/rocm/include/"  ; rm -fR $(@D)/rocm/include/gtest ; rm -fR $(@D)/rocm/include/gmock""",
)
genrule(
    name = "rocm-lib",
    outs = [
        "rocm/lib/libamdhip64.so",
    ],
    cmd = """cp -f "/opt/rocm-5.0.0/hip/lib/libamdhip64.so.5.0.50000" "$(location rocm/lib/libamdhip64.so)" """,
)
genrule(
    name = "rocm-bin",
    outs = [
        "rocm/bin/clang-offload-bundler",
    ],
    cmd = """cp -f "/opt/rocm/llvm/bin/clang-offload-bundler" "$(location rocm/bin/clang-offload-bundler)" """,
)
