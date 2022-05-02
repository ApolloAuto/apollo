# Macros for building ROCm code.
def if_rocm(if_true, if_false = []):
    """Shorthand for select()'ing on whether we're building with ROCm.

    Returns a select statement which evaluates to if_true if we're building
    with ROCm enabled. Otherwise, the select statement evaluates to if_false.

    """
    return select({
        "@local_config_rocm//rocm:using_hipcc": if_true,
        "//conditions:default": if_false
    })

def if_hip_clang_opt(if_true, if_false = []):
   """Shorthand for select()'ing on wheteher we're building with hip-clang
   in opt mode.

    Returns a select statement which evaluates to if_true if we're building
    with hip-clang in opt mode. Otherwise, the select statement evaluates to
    if_false.

   """
   return select({
       "@local_config_rocm//rocm:using_clang_opt": if_true,
       "//conditions:default": if_false
   })

def hip_default_copts():
    """Default options for all HIP compilations."""
    return if_hip([
        "-x", "hip"
    ])
    + if_hip_clang_opt(
        # Some important HIP optimizations are only enabled at O3.
        ["-O3"]
    )

def rocm_header_library(
        name,
        hdrs,
        include_prefix = None,
        strip_include_prefix = None,
        deps = [],
        **kwargs):
    """Generates a cc_library containing both virtual and system include paths.

    Generates both a header-only target with virtual includes plus the full
    target without virtual includes. This works around the fact that bazel can't
    mix 'includes' and 'include_prefix' in the same target."""

    native.cc_library(
        name = name + "_virtual",
        hdrs = hdrs,
        include_prefix = include_prefix,
        strip_include_prefix = strip_include_prefix,
        deps = deps,
        visibility = ["//visibility:private"],
    )

    native.cc_library(
        name = name,
        textual_hdrs = hdrs,
        deps = deps + [":%s_virtual" % name],
        **kwargs
    )

def rocm_library(copts = [], **kwargs):
    """Wrapper over cc_library which adds default HIP options."""
    native.cc_library(copts = hip_default_copts() + copts, **kwargs)
