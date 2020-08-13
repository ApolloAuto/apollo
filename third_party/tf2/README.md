# tf2-apollo

## Intro

tf2-apollo was Apollo's clone of ROS tf2 for coordinate transforms.
It seems that it was based on `ros/geometry2` tagged `0.5.16`

## How to build

```
mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local/tf2
make -j$(nproc)
sudo make install
```

## gtest support for tf2-apollo

In order not to build gtest into our docker image, gtest support for tf2 was
implemented according to https://crascit.com/2015/07/25/cmake-gtest

## Reference
- https://github.com/ros/geometry2
- http://wiki.ros.org/geometry2

