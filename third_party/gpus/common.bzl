load("@local_config_cuda//cuda:build_defs.bzl", "cuda_default_copts")

def if_cuda(if_true, if_false = []):
    return select({
        "@local_config_cuda//cuda:using_nvcc": if_true,
        "@local_config_cuda//cuda:using_clang": if_true,
        "//conditions:default": if_false,
    })

def if_rocm(if_true, if_false = []):
    return select({
        "@local_config_rocm//rocm:using_hipcc": if_true,
        "//conditions:default": if_false
    })

def if_cuda_clang(if_true, if_false = []):
   return select({
       "@local_config_cuda//cuda:using_clang": if_true,
       "//conditions:default": if_false
   })

def if_cuda_clang_opt(if_true, if_false = []):
   return select({
       "@local_config_cuda//cuda:using_clang_opt": if_true,
       "//conditions:default": if_false
   })

def if_rocm_clang_opt(if_true, if_false = []):
   return select({
       "@local_config_rocm//rocm:using_clang_opt": if_true,
       "//conditions:default": if_false
   })

def rocm_default_copts():
    return if_rocm(
        ["-x", "hip"]
    )
    + if_rocm_clang_opt(
        ["-O3"]
    )

def gpu_default_copts():
    return cuda_default_copts() + rocm_default_copts()

def gpu_library(copts = [], **kwargs):
    native.cc_library(copts = gpu_default_copts() + copts, **kwargs)
