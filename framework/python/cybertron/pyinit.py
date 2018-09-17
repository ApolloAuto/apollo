import time
import sys
import os
import importlib

# init vars
cybertron_path = os.environ['CYBERTRON_PATH']
if (cybertron_path == ""):
    print("CYBERTRON_PATH is null")
else:
    print("CYBERTRON_PATH=%s" % cybertron_path)
    print("env inited succ!")

cybertron_dir=os.path.split(cybertron_path)[0]
sys.path.append(cybertron_path + "/third_party/")
sys.path.append(cybertron_path + "/lib/")
sys.path.append(cybertron_path + "/python/cybertron")

sys.path.append(cybertron_dir + "/python/")
sys.path.append(cybertron_dir + "/cybertron/")

_cyber_init = importlib.import_module('_cyber_init')

def init():
    return _cyber_init.py_init()

def ok():
    return _cyber_init.py_ok()

def shutdown():
    return _cyber_init.py_shutdown()

def is_shutdown():
    return _cyber_init.py_is_shutdown()

def waitforshutdown():
    return _cyber_init.py_waitforshutdown()
