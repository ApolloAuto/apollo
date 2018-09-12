import time
import sys
import os


# first excute for init vars
cybertron_path = os.environ['CYBERTRON_PATH']
if (cybertron_path == ""):
    print("CYBERTRON_PATH is null") 
else:
    print("CYBERTRON_PATH=%s" % cybertron_path)
    print("env inited succ!") 

cybertron_dir = os.path.split(cybertron_path)[0]
sys.path.append(cybertron_dir + "/python/")

from cybertron import pyinit
    
def main():
    pyinit.init()
    if pyinit.ok() :
        print("init success")
    else:
        print("init failed")
    pyinit.shutdown()
    if pyinit.is_shutdown() :
        print("shutdown success")
    else:
        print("shutdown failed")

if __name__ == '__main__':
  main()