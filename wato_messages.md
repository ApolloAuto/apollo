
# Driver messages that need to be mapped

## Camera Image

This is the basic image message sent by any one of the cameras. They come in two types: raw images (Image) or png/jpg (CompressedImage).

I think we will only be sending raw images (need to investigate whether we *can* do this or if it will break things by not sending a CompressedImage).

TODO: need to find topics that are associated with this message

> http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
> https://github.com/WATonomous/apollo/blob/master/modules/drivers/proto/sensor_image.proto
/camera/right/image_color sensor_msgs/Image -> [TODO: channel] Image

```protobuf
message Image {
  optional apollo.common.Header header = 1; // (see below)

  optional string frame_id = 2; // maps to header.frame_id
  optional double measurement_time = 3; // maps to header.timestamp

  optional uint32 height = 4; // maps to height
  optional uint32 width = 5; // maps to width

  optional string encoding = 6; // maps to encoding
  optional uint32 step = 7; // maps to step
  optional bytes data = 8; // maps to data
}

message Header {
  optional double timestamp_sec = 1; // maps to header.timestamp
  optional string module_name = 2; // maps to "Camera"
  optional uint32 sequence_num = 3; // TODO: not sure

  // All timestamps in nanoseconds
  optional uint64 lidar_timestamp = 4; // ignore
  optional uint64 camera_timestamp = 5; // maps to header.timestamp
  optional uint64 radar_timestamp = 6; // ignore

  optional uint32 version = 7 [default = 1]; // leave default?
  optional StatusPb status = 8; // ignore
  optional string frame_id = 9; // maps to header.frame_id
}
```


## Lidar PointCloud



