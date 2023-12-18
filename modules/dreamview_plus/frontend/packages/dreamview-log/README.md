# `@dreamview/log`

> description

这个模块是一个用于创建和管理日志的库。它基于开源库 loglevel 来实现日志记录功能，并且提供了一个 Logger 类，用于创建和管理各种不同的日志记录器。

Logger 类的每个实例都可以独立地设置日志级别，并且可以输出带有时间戳和日志级别的日志消息。这个类是单例的，也就是说，对于每个给定的名称，Logger.getInstance(name) 始终返回同一个 Logger 实例。这个实例在第一次请求时创建，并在后续的请求中重复使用。




## Usage

```
const logger = Logger.getInstance('myLogger');
const logger = Logger.getInstance(__filename);

// DEMONSTRATE API
logger.debug('This is a debug message');
logger.info('This is an info message');
logger.warn('This is a warning message');
logger.error('This is an error message');
```
