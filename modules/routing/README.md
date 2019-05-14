# Routing

## Introduction
  The Routing module generates high level navigation information based on requests.

  The Routing module depends on a routing topology file, usually named `routing_map.*` in apollo.

  The routing map can be genreated using this command:
  ```
  bash scripts/generate_routing_topo_graph.sh
  ```

## Input
  * Map data
  * Routing request (Start and end location)

## Output
  * Routing navigation information
