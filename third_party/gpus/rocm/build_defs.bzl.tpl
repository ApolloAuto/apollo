# Macros for building ROCm code.
def if_rocm(if_true, if_false = []):
    return select({
        "@local_config_rocm//rocm:using_rocm": if_true,
        "//conditions:default": if_false
    })


def rocm_default_copts():
    return if_rocm(["", ""])

def rocm_copts(opts = []):
    return rocm_default_copts() + select({
        "//conditions:default": [],
        "@local_config_rocm//rocm:using_rocm": ([
            "",
        ]),
    }) + if_rocm_is_configured(opts)

def rocm_library(copts = [], **kwargs):
    """Wrapper over cc_library which adds default ROCm options."""
    native.cc_library(copts = rocm_default_copts() + copts, **kwargs)
