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

cybertron_dir=os.path.split(cybertron_path)[0]
sys.path.append(cybertron_path + "/third_party/")
sys.path.append(cybertron_path + "/lib/")
sys.path.append(cybertron_path + "/python/cybertron")

sys.path.append(cybertron_dir + "/python/")
sys.path.append(cybertron_dir + "/cybertron/")

from cybertron import pyrecord

def test_record_writer():
    fw = pyrecord.PyRecordFileWriter()

def main():
    test_record_writer()

if __name__ == '__main__':
  main()