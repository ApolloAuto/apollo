# `dreamview-mock`

> `dreamview-mock`是一个用于模拟Dreamview WebSocket服务器的Node.js模块。它提供了一个便捷的方法，使开发者能够快速添加自己的实际业务接口。

## Usage

在你的项目中，你可以直接使用dreamview-mock命令启动模拟服务器：
```bash
yarn lerna run dreamview-mock
```
服务器会自动启动，并监听指定的端口。

## Capacity
`dreamview-mock`提供了两种通讯方式：

发布订阅模式：当客户端连接到服务器时，可以向服务器发送subscribe消息来订阅某种类型的数据。服务器会定时向所有已订阅的客户端发送这种类型的数据。客户端也可以发送unsubscribe消息来取消订阅。

请求响应模式：客户端可以发送带有requestId的请求，服务器会在响应中返回相同的requestId。这模拟了类似HTTP的请求响应模式。

# `dreamview-recorder`

> `dreamview-recorder`是一个用于记录Dreamview WebSocket服务器数据的Node.js模块。它提供了一个便捷的方法，使开发者能够快速记录自己的实际业务接口。

## Usage

命令行工具：
```bash
// 查看帮助
yarn workspace @dreamview/dreamview-mock dreamview-record -H
```
