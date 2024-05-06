# drivers-image-decompress

## Introduction
This package is responsible for decompressing the image in jpeg encoding to rgb format.

## Directory Structure
```shell
modules/drivers/tools/image_decompress
|-- BUILD
|-- README.md
|-- conf
|-- cyberfile.xml
|-- dag
|-- drivers-image-decompress.BUILD
|-- image_decompress.cc
|-- image_decompress.h
|-- image_decompress_test.cc
|-- launch
`-- proto
```

## Modules

### ImageDecompressComponent

apollo::image_decompress::ImageDecompressComponent


#### Input

| Name  | Type                                               |      Description             |
| ----- | -------------------------------------------------- | ---------------------------- |
| `msg` |         `apollo::drivers::CompressedImage`         |    image in jpeg encoding    |

#### Output

| Name  | Type                                               |      Description             |
| ----- | -------------------------------------------------- | ---------------------------- |
| `msg` |             `apollo::drivers::Image`               |      image in RGB format     |

#### configs

| file path                                                              | type / struct                       | Description           |
| ---------------------------------------------------------------------- | ----------------------------------- | --------------------- |
| `modules/drivers/tools/image_decompress/conf/camera_front_6mm.pb.txt`  |  `apollo::image_decompress::Config` |image decompress config|

#### How to Launch

```bash
cyber_launch start modules/drivers/video/launch/video.launch
```
