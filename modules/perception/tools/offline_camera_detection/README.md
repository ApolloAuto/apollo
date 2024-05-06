## offline_camera_detection
`offline_camera_detection` is Apollo's camera offline detection tool.

## How to work
1. First, build the perception module
```
./apollo.sh build perception
```
2. Then run the tool
```
bash modules/perception/tools/offline_camera_detection/offline_camera_detection.sh
```
3. Finally, view the results in `/apollo/data`
```
test.jpg  test.txt
```

## Input
The input image data is in the `images` directory. You need to put the test pictures into this directory, and add the picture name in `images\image_test_list.txt`.

The default test image is `images\test.jpg`, and `images\image_test_list.txt` contains the name **test** in it.

## Output
The output data is in `/apollo/data`, Include text files and image files.

The default test result is `test.jpg  test.txt`.

## Model config
The model config file is in `conf\camera_detection_pipeline.pb.txt`, which you can change the model.

The default detection model is `SMOKE_OBSTACLE_DETECTION`.

## Parameters
`offline_camera_detection` supports the following parameters
- height, Input image height
- width, Input image width
- image_ext, Input image format
