package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "adv_plat",
    srcs = [
        "lib/libadv_bcan.so.1",
        "lib/libadv_trigger.so.1",
    ],
    hdrs = [
        "include/adv_trigger.h",
        "include/bcan.h",
        "include/linux/bcan_defs.h",
        "include/linux/zynq_api.h",
    ],
    include_prefix = "adv_plat",
)
