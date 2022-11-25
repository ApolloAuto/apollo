def if_teleop(if_true, if_false = []):
    return select({
        "//tools/platform:with_teleop": if_true,
        "//conditions:default": if_false,
    })

def copts_if_teleop():
    return if_teleop(["-DWITH_TELEOP=1"], ["-DWITH_TELEOP=0"])

def if_x86_64(if_true, if_false = []):
    return select({
        "@platforms//cpu:x86_64": if_true,
        "//conditions:default": if_false,
    })

def if_aarch64(if_true, if_false = []):
    return select({
        "@platforms//cpu:aarch64": if_true,
        "//conditions:default": if_false,
    })

def if_esd_can(if_true, if_false = []):
    return select({
        "//tools/platform:use_esd_can": if_true,
        "//conditions:default": if_false,
    })

def copts_if_esd_can():
    return if_esd_can(["-DUSE_ESD_CAN=1"], ["-DUSE_ESD_CAN=0"])

