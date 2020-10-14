# How to Generate Local Map For MSF Localization Module

## Prerequisites
 - Download source code of Apollo from [GitHub](https://github.com/ApolloAuto/apollo)
 - Follow the tutorial to set up [docker environment](../quickstart/apollo_software_installation_guide.md).
 - ~~Download localization data from the [Multi-Sensor Fusion Localization Data](http://data.apollo.auto/help?name=sensor%20data&data_key=multisensor&data_type=1&locale=en-us&lang=en)（US only).~~
 - Download localization dataset: please contact Yao Zhou, zhouyao@baidu.com, to request the dataset. Requests need contain the following: (1) Email address and affiliation (business or school); (2) Application purpose.

## Build Apollo

First check to make sure you are in development docker container before you proceed. Now you will need to build from source:
```
# To make sure you start clean
bash apollo.sh clean

# Build the full system
bash apollo.sh build_opt
```

## Generate Localization Map
In the downloaded data, look for a folder named *apollo3.5*. Let's assume the path of this folder to be DATA_PATH.

```
/apollo/scripts/msf_simple_map_creator.sh DATA_PATH/records/ DATA_PATH/params/velodyne_params/velodyne128_novatel_extrinsics.yaml 10 OUT_FOLDER_PATH
```

After the script is finished, you can find the produced localization map named *local_map* in the output folder.

The scripts also stores the visualization of each generated map node in the map's subfolder named `image`. The visualization of a map node filled with LiDAR data looks like this:

![1](images/msf_localization/map_node_image.png)
