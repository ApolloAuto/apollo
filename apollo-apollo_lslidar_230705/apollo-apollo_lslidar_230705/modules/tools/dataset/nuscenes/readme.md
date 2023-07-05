## Convert dataset
You can use below command to convert nuscenes dataset to apollo record file. There maybe multi sense in one dataset, and we create a record file for each scene.

```shell
python3 main.py -i nuscenes_dataset_path
```
The name of the record file is the `scene_token.record`. If you do not specify a path, the file will be saved in the current path.

## Convert calibration
You can use below command to convert nuscenes calibration to apollo calibration files. There maybe multi sense in one dataset, and we create calibration files for each scene.

```shell
python3 main.py -i nuscenes_dataset_path -t=cal
```

#### Camera intrinsics
Camera intrinsics matrix. ref [link](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)
- D: The distortion parameters, size depending on the distortion model. For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
- K: Intrinsic camera matrix for the raw (distorted) images.
- R: Rectification matrix (stereo cameras only)
- P: Projection/camera matrix

## Convert lidar pcd
You can use below command to convert nuscenes lidar pcd to normal pcl file, which you can display in visualizer like `pcl_viewer`.

```shell
python3 main.py -i nuscenes_lidar_pcd_file -t=pcd
```
If you do not specify a name, the default name of the file is `result.pcd`, which is saved in the current directory.
