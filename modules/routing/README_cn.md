# 路由

## 介绍

路由模块根据请求生成高级导航信息。

路由模块依赖于路由拓扑文件，通常称为Apollo中的routing_map.*。路由地图可以通过命令来生成。
```
bash scripts/generate_routing_topo_graph.sh
```
## 输入

* 地图数据
* 路由请求（开始和结束位置）
## 输出

* 路由导航信息
