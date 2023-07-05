load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
  
def init():
    cuda_configure(name = "local_config_cuda")