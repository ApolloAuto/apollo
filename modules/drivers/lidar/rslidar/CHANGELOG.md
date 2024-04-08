# Changelog

## v1.3.1 - 2022-10-09

### Fixed

- Fix point cloud publish error when multiple LiDAR drivers start in one dag

## v1.3.0 - 2022-08-08

### Added

- Add the functions of subscribing Apollo type packet messages and decoding point clouds from them, which are implemented at `/apollo/modules/drivers/lidar/robosense/test`

## v1.2.0 - 2022-08-04

### Added

- Add the function of publishing Apollo type packet messages

## v1.1.0 - 2022-08-01

### Added

- Refactor RobosenseDriver by using new interfaces in rs_driver v1.5.4 

### Fixed

- Fix the wrong path in launch and dag files in README_CN and README

## v1.0.0 - 2020-07-28

### Added

- LiDAR Driver on Apollo platform

- Add RS16, RS32, RS128, RSBP support


