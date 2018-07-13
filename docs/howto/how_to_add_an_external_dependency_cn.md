## 如何添加新的外部依赖项
本文解释了设计和实现的目标是最小化系统中必须预先安装的依赖性的数目。如果您的要添加的目标依赖于一个必须先 `apt-get install`的模块，建议您用 **Bazel**作为包/依赖性的管理系统

例如，如果您想添加一个不是最初就用Bazel构建的工作站规则 `foo`，操作过程如下：

- 在WORKSPACE中添加一个名为 'foo'的工作站规则
- 明确 `foo`的来源(通常是一个URL)和版本(通常是git的标签或commit的哈希码).
- 为了构建，在third_party目录下新建一个 `foo.BUILD`。 这个 BUILD 文件和您的目标中的其它的Bazel BUILD文件一样。
- 在您依赖于 `foo`的目标中，把 `@foo://:<foo_target>` 加入到依赖项中。

### 用Bazel添加一个外部依赖项

如果用Bazel把 `foo`添加进工作站规则中来构建目标，依赖于 `foo`，Bazel把 `foo`的源码从指定的源中拉出来，并以 `foo.BUILD`构建它。如果原来就用Bazel构建的，那么只需要工作站规则。

### 参考

有关用Bazel添加依赖项的详细描述，请参阅以下内容： 
* [工作站规则](https://bazel.build/versions/master/docs/be/workspace.html)
* [在有外部依赖项的情况下工作](https://docs.bazel.build/versions/master/external.html).
