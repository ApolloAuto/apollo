# Routing

## Introduction
  The Routing module generates high level navigation information based on requests.

  The Routing module depends on a routing topology file, usually named `routing_map.*` in apollo.

  The routing map can be genreated using this command:
  ```
  bash scripts/generate_routing_topo_graph.sh
  ```

## Directory Structure
```shell
modules/routing/
├── BUILD
├── common
├── conf
├── core
├── cyberfile.xml
├── dag
├── graph
├── launch
├── proto
├── README_cn.md
├── README.md
├── routing.cc
├── routing_component.cc
├── routing_component.h
├── routing.h
├── strategy
├── tools
└── topo_creator
```

## Input
  * Map data
  * Routing request (Start and end location)

## Output
  * Routing navigation information

 
## configs

| file path                                    | type / struct                        | Description           |
| -------------------------------------------- | ------------------------------------ | --------------------- |
| `modules/routing/conf/routing_config.pb.txt` | `apollo::routing::RoutingConfig`     | routing config        |
| `modules/routing/conf/routing.conf`          | `gflags`                             |   gflags config       |

## Flags

| flagfile                                  | type | Description                |
| ----------------------------------------- | ---- | -------------------------- |
| `modules/routing/common/routing_gflags.cc`| `cc` | routing flags define       |
| `modules/routing/common/routing_gflags.h` | `h`  | routing flags header       |


#### How to Launch

```bash
cyber_launch start modules/routing/launch/routing.launch
```
