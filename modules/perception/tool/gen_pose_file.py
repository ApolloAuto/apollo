#!/usr/bin/python
import sys
import os

def main(argv):
    assert len(argv) <= 2, "The number of input parameter should be empty or only one"
    pcd_dir = "/apollo/data/pcd"   # default PCD file directory
    if len(argv) == 2:
        pcd_dir = argv[1]

    pose_dir = os.path.join(os.path.dirname(pcd_dir), "pose")   # Pose file directory
    if not os.path.exists(pose_dir):
        os.mkdir(pose_dir)

    pose_info_file = os.path.join(pcd_dir, "pose.txt")
    assert os.path.exists(pose_info_file), "The file of pose info does not exist"
    with open(pose_info_file, 'r') as f_in:
        for line in f_in:
            out_pose_file = os.path.join(pose_dir, line.split()[0] + ".pose")
            with open(out_pose_file, 'w') as f_out:
                print >> f_out, line

if __name__ == "__main__":
    main(sys.argv)