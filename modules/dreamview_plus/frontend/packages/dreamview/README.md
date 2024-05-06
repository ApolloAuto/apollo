# `dreamview`

> Declaration file and proto type for dreamview data

## Usage
生成的JavaScript文件（dreamview_proto_bundle.js）是一个 CommonJS 模块，可以在 Node.js 环境中或者在支持 CommonJS 的前端打包工具（如 webpack、Browserify）中使用。

```
const protobuf = require('dreamview');

// 现在你可以使用 protobuf 对象来访问你的 .proto 文件中定义的所有方法和类型。
// 例如，假设你有一个名为 `planning` 的消息类型：
let message = protobuf.planning.create({ /* 你的数据 */ });

// 序列化消息到二进制格式
let buffer = protobuf.planning.encode(message).finish();

// 从二进制格式反序列化消息
let decodedMessage = protobuf.planning.decode(buffer);
```
