cc_library(
    name = "external_command_processor_base",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/lib*.so*"]),
    include_prefix = "modules/external_command/command_processor/command_processor_base",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)