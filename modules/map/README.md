# map

## Introduction
Map module is an important part of Apollo, which is mainly used to load map data in OpenStreetMap format and provides rich map operation APIs, including map drawing, positioning, search and other functions. Through the Apollo Map module, the vehicle can obtain real-time road information, traffic conditions, point-of-interest information, etc., so as to carry out intelligent path planning, navigation and other operations.

## Directory Structure

```shell
modules/map/
├── BUILD
├── cyberfile.xml
├── data                // Map data, including vector maps, satellite maps and so on.
├── hdmap               // API implementations of high-precision maps.
├── pnc_map             // API implementations for the planning control module.
├── README.md
├── relative_map        // API implementations for relative map. 
├── testdata
└── tools               // tools for processing map data and performing map-related operations.
```

