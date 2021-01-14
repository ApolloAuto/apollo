#!/usr/bin/env python3

import os
import glob

## == Constant Definitions ==##
TAB = " " * 4

top_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

def guess_solib_dir():
    dirs = glob.glob(os.path.join(top_dir, "bazel-bin/_solib_*"))
    assert len(dirs) == 1, "Only one _solib_* directory under bazel-bin is expected"
    return dirs[0]

solib_dir = guess_solib_dir()

def install_local_solibs():
    relative_dir = solib_dir[len(top_dir)+1:]
    for lib in os.listdir(solib_dir):
        if os.path.isfile(os.path.join(solib_dir, lib)):
            print("{}install(\"{}/{}\", \"lib/{}\")".format(TAB, relative_dir, lib, lib))

def install_data(src, dst):
    print("{}install(\"{}\", \"{}\")".format(TAB, src, dst))
 
def install_cyber():
    cyber_conf = "cyber/conf/cyber.pb.conf"
    install_data(cyber_conf, cyber_conf)
    dreamview_conf = "cyber/conf/dreamview_sched.conf"
    install_data(dreamview_conf, dreamview_conf)
    install_data("bazel-bin/cyber/mainboard/mainboard", "bin/mainboard")

def install_cyber_examples():
    # Common Component Example
    install_data("bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so",
            "cyber/examples/common_component_example/libcommon_component_example.so")
    install_data("cyber/examples/common_component_example/common.dag", "cyber/examples/common_component_example/common.dag")
    install_data("cyber/examples/common_component_example/common.launch", "cyber/examples/common_component_example/common.launch")

    # Timer Component Example
    install_data("bazel-bin/cyber/examples/timer_component_example/libtimer_component_example.so", "cyber/examples/timer_component_example/libtimer_component_example.so")
    install_data("cyber/examples/timer_component_example/timer.dag", "cyber/examples/timer_component_example/timer.dag")
    install_data("cyber/examples/timer_component_example/timer.launch", "cyber/examples/timer_component_example/timer.launch")

def main():
    install_local_solibs()
    install_cyber()
    install_cyber_examples()

if __name__ == "__main__":
    main()
