#!/usr/bin/env python3

import os
import sys

top_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
install_dir = os.path.join(top_dir, "tools/install")

strs = None

with open(os.path.join(install_dir, "abc.txt")) as f:
    strs = f.readlines()
os.remove(os.path.join(install_dir, "abc.txt"))

with open(os.path.join(install_dir, "install.py"), "w") as fout:
    with open(os.path.join(install_dir, "install.py.in")) as fin:
        for line in fin:
            text = line.strip()
            if text == "<<actions>>":
                for sl in strs:
                    fout.write(sl)
            else:
                fout.write(line)

