## Convert dataset
`adataset` is used to convert datasets (nuScenes, KITTI, ApolloScape) to Apollo record file. This way we can guarantee **the consistency of training data and test data**, including sensor intrinsics and extrinsics parameter files, thus speeding up model validation.

## Install
```
pip3 install adataset
```

## Usage
We first introduce the use of the command, and then introduce how to use the dataset with `adataset`.

#### Command options
The options for `adataset` command are as follows:
* --dataset(-d) Choose the dataset, support list `n, k, a, w`, means "n:nuScenes, k:KITTI, a:ApolloScape, w:Waymo"
* --input(-i) Set the dataset input directory.
* --output(-o) Set the output directory, default is the current directory.
* --type(-t) Choose conversion type, support list `rcd, cal, pcd`, means "rcd:record, cal:calibration, pcd:pointcloud", default is `rcd`.

#### Convert record files
You can use below command to convert dataset to Apollo record file. For example convert nuScenes dataset in `dataset_path` to Apollo record. The `output` default is the current directory, and the `type` default is `rcd`.
```shell
adataset -d=n -i=dataset_path
```
The name of the nuScenes record file is `scene_token.record`, and KITTI is `result.record`, and ApolloScape is `frame_id.record`

#### Convert calibration files
You can use below command to convert dataset to apollo calibration files. There maybe multi sense in one dataset, and we create calibration files for each scene.
```shell
adataset -d=n -i=dataset_path -t=cal
```

###### Camera intrinsics
Camera intrinsics matrix. ref [link](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)
- D: The distortion parameters, size depending on the distortion model. For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
- K: Intrinsic camera matrix for the raw (distorted) images.
- R: Rectification matrix (stereo cameras only)
- P: Projection/camera matrix

#### Convert PCD file
You can use below command to convert dataset lidar pcd to normal pcl file, which can display in visualization tools such as `pcl_viewer`.
```shell
adataset -d=n -i=dataset_lidar_pcd_file -t=pcd
```
If you do not specify a name, the default name of the pcd file is `result.pcd`, saved in the current directory.

## Dataset introduction
There are differences between the data sets, so introduce them separately.

#### nuScenes
[nuScenes Mini](https://www.nuscenes.org/nuscenes#download) compared with the full amount of data, the Mini data set is relatively small. The nuScenes Mini data set is as follows.
```
nuScenes-Mini
 -maps
 -samples
 -sweeps
 -v1.0-mini
```
Then we can use the following command to generate the "record/calibration/pcd" file.
```
// record
adataset -d=n -i=path/to/nuScenes-Mini
// calibration
adataset -d=n -i=path/to/nuScenes-Mini -t=cal
// pcd
adataset -d=n -i=path/to/nuScenes-Mini/samples/LIDAR_TOP/n015-2018-11-21-19-38-26+0800__LIDAR_TOP__1542801007446751.pcd.bin -t=pcd
```

#### KITTI
We use [KITTI raw data](https://www.cvlibs.net/datasets/kitti/raw_data.php) to generate Apollo record file. Be sure to download `[synced+rectified data]` but not `[unsynced+unrectified data]`. Note that the calibration data are in `[calibration]`.

###### dataset
The KITTI raw data is as follows.
```
2011_09_26_drive_0015_sync
 -image_00
 -image_01
 -image_02
 -image_03
 -oxts
 -velodyne_points
```
Then we can use the following command to generate the "record/pcd" file.
```
// record
adataset -d=k -i=path/to/2011_09_26_drive_0015_sync
// pcd
adataset -d=k -i=path/to/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data/0000000113.bin -t=pcd
```

###### calibration
The KITTI calibration data is as follows:
```
2011_09_26
 -calib_cam_to_cam.txt
 -calib_imu_to_velo.txt
 -calib_velo_to_cam.txt
```
Then we can use the following command to generate the Apollo "calibration" files.
```
adataset -d=k -i=path/to/2011_09_26 -t=cal
```

#### ApolloScape
We use [ApolloScape](https://apolloscape.auto/) Detection/Tracking dataset to generate record file.The ApolloScape Detection/Tracking data is as follow.
```
Training data
 -detection_train_pcd_1.zip
 -detection_train_bin_1.zip
 -detection_train_label.zip
 -...
 -tracking_train_pose.zip

Testing data
 -detection_test_pcd_1.zip
 -detection_test_bin_1.zip
 -...
 -tracking_test_pose.zip
```

Before generating record file, we should organize data folders as follow(use test data for example).
```
├── tracking_test
    ├── pcd
        ├── result_9048_2_frame
            ├── 2.bin
            ├── 7.bin
            └── ...
        └── ...
    ├── pose
        ├── result_9048_2_frame
            ├── 2_pose.txt
            ├── 7_pose.txt
            └── ...
        └── ...
```

Then we can use the following command to generate the "record/calibration/pcd" file.
```
// record
python main.py -d=a -i=tracking_test/ -o=records/ -t=rcd
// calibration
python main.py -d=a -i=tracking_test/ -o=records/ -t=cal
// pcd
python main.py -d=a -i=data_path/data.bin -o=data_path/result.pcd -t=pcd
```
