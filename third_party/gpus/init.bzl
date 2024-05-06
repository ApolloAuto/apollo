load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
load("//third_party/gpus:rocm_configure.bzl", "rocm_configure")
  
def init():
    cuda_configure(name = "local_config_cuda")
    rocm_configure(name = "local_config_rocm")
