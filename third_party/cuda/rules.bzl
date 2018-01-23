# -*- Python -*-


# Given a source file, generate a test name.
# i.e. "common_runtime/direct_session_test.cc" becomes
#      "common_runtime_direct_session_test"


load("@local_config_cuda//cuda:build_defs.bzl", "if_cuda", "cuda_default_copts")
# Sanitize a dependency so that it works correctly from code that includes
# TensorFlow as a submodule.
def clean_dep(dep):
  return str(Label(dep))

# LINT.IfChange
def tf_copts():
  return ([
      "-DEIGEN_AVOID_STL_ARRAY",
      "-Iexternal/gemmlowp",
      "-Wno-sign-compare",
      "-fno-exceptions",
  ] + if_cuda(["-DGOOGLE_CUDA=1"]))


def _cuda_copts():
  """Gets the appropriate set of copts for (maybe) CUDA compilation.

    If we're doing CUDA compilation, returns copts for our particular CUDA
    compiler.  If we're not doing CUDA compilation, returns an empty list.

    """
  return cuda_default_copts() + select({
      "//conditions:default": [],
      "@local_config_cuda//cuda:using_nvcc": ([
          "-nvcc_options=relaxed-constexpr",
          "-nvcc_options=ftz=true",
      ]),
      "@local_config_cuda//cuda:using_clang": ([
          "-fcuda-flush-denormals-to-zero",
      ]),
  })

def tf_cuda_library(deps=None, cuda_deps=None, copts=None, **kwargs):
  """Generate a cc_library with a conditional set of CUDA dependencies.

  When the library is built with --config=cuda:

  - both deps and cuda_deps are used as dependencies
  - the cuda runtime is added as a dependency (if necessary)
  - The library additionally passes -DGOOGLE_CUDA=1 to the list of copts

  Args:
  - cuda_deps: BUILD dependencies which will be linked if and only if:
      '--config=cuda' is passed to the bazel command line.
  - deps: dependencies which will always be linked.
  - copts: copts always passed to the cc_library.
  - kwargs: Any other argument to cc_library.
  """
  if not deps:
    deps = []
  if not cuda_deps:
    cuda_deps = []
  if not copts:
    copts = []

  native.cc_library(
      deps=deps + if_cuda(cuda_deps + [
          # clean_dep("//tensorflow/core:cuda"),
          "@local_config_cuda//cuda:cuda_headers"
      ]),
      copts=copts + if_cuda(["-DGOOGLE_CUDA=1"]),
      **kwargs)

# When this target is built using --config=cuda, a cc_library is built
# that passes -DGOOGLE_CUDA=1 and '-x cuda', linking in additional
# libraries needed by GPU kernels.
def tf_gpu_kernel_library(srcs,
                          copts=[],
                          cuda_copts=[],
                          deps=[],
                          hdrs=[],
                          **kwargs):
  copts = copts + _cuda_copts() + if_cuda(cuda_copts) + tf_copts()

  native.cc_library(
      srcs=srcs,
      hdrs=hdrs,
      copts=copts,
      deps=deps + if_cuda([
          # clean_dep("//tensorflow/core:cuda"),
          # clean_dep("//tensorflow/core:gpu_lib"),
          "@local_config_cuda//cuda:cuda_headers"
      ]),
      alwayslink=1,
      **kwargs)

