## How to use Lego config system in Planning?

Apollo Lego Config contains two types of conf files in Planning, namely gflags and protobufs. The gflag files are contained in the modules/planning/common directory. The protobuf config files are in the modules/planning/conf directory. When you set the config items using dreamview, the config system will modify the values in gflag files or *.pb.txt files. Thus, the system should use your new values after the module is restarted.



## What is the meaning of the config items?

Before you start to change the config items, you should have a better understand of the whole algorithm. 

The gflag items are defined and explained in modules/planning/common/planning_gflags.cc. As examples, here are a few items that are frequently used while testing:

* planning_upper_speed_limit: Maximum speed (m/s) in planning
* default_cruise_speed: default cruise speed

Note that not every item in *_gflag.cc is in planning/conf/\*.conf, which only contains the most frequently changed items. Feel free to change the items from dreamview if they don't exist in planning/conf/\*.conf files.

For the *.pb.txt files, you can find the whole format in the modules/planning/profo directory. Config items in *.pb.txt are often relative to some certain algorithms. So make sure you know the tasks before modifying them.



## How does the change take effect?

By setting new values in dreamview, the items in *.conf will be changed or added. Or configs in *.pb.txt will be changed. So, it will take effect when you restart the planning module.