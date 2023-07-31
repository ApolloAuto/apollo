cc_library(
    name = "external_command_process",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/lib*.so*"]),
    include_prefix = "modules/external_command/process_component",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)