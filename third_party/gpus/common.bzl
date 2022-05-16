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

# TODO(emankov): Don't forget to add %{cuda_extra_copts} and %{rocm_extra_copts} after setting them in the corresponding _create_***_repository() functions
def gpu_default_copts():
    return if_cuda([
        "-x", "cuda",
        "-DAPOLLO_CUDA=1",
        "-Xcuda-fatbinary=--compress-all",
        "--no-cuda-include-ptx=all"
    ]) + if_cuda_clang_opt(
        ["-O3"]
    ) + if_rocm(
        ["-x", "hip"]
    ) + if_rocm_clang_opt(
        ["-O3"]
    )

def gpu_library(copts = [], **kwargs):
    native.cc_library(copts = gpu_default_copts() + copts, **kwargs)
