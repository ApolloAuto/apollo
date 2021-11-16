# Apollo 编码最佳实践

1. 提交 PR 前记得先在本地通过编译、单元测试和代码检查。

   ```bash
   ./apollo.sh check
   ```

1. 请写单元测试，并随源文件一起提交。

   ```text
   foobar.h
   foobar.cc
   foobar_test.cc
   ```

1. 一个 Bazel 目标（Target）最多包含一个头文件和一个（`.cc`）源文件。

   ```python
   cc_library(
     name = "foobar",
     hdrs = ["foobar.h"],
     srcs = ["foobar.cc"],
     deps = [
       ...
     ],
   )

   cc_test(
     name = "foobar_test",
     srcs = ["foobar_test.cc"],
     deps = [
       ":foobar",
       ...
     ]
   )
   ```

   可运行 `./apollo.sh format <path/to/BUILD>` 来修复 BUILD 文件的格式问题。

1. 总体上，Apollo 遵循
   [Google C++风格指南](https://google.github.io/styleguide/cppguide.html).
   通过运行`scripts/clang_format.sh <path/to/cpp/dirs/or/files>` 或
   `./apollo.sh format -c <path/to/cpp/dirs/or/files>` 命令可修复
   C++代码风格问题。

1. 确保简单且一致的函数签名。注释中请不要出现中文。

   ```C++
   // 1. For input objects, const reference guarantes that it is valid, while
   //    pointers might be NULL or wild. Don't give others the chance to break
   //    you.
   // 2. For input scalars, just pass by value, which gives better locality and
   //    thus performance.
   // 3. For output, it's the caller's responsibility to make sure the pointer
   //    is valid. No need to do sanity check or mark it as "OutputType* const",
   //    as pointer redirection is never allowed.
   void FooBar(const InputObjectType& input1, const InputScalaType input2, ...,
               OutputType* output1, ...);

   // RVO machanism will help you avoid unnecessary object copy.
   // See https://en.wikipedia.org/wiki/Copy_elision#Return_value_optimization
   OutputType FooBar(const InputType& input);
   ```

1. 尽可能使用`const` 修饰变量，函数。

   ```C++
   // Variables that don't change.
   const size_t current_size = vec.size();
   // Functions that have no side effect.
   const std::string& name() const;
   ```

1. 尽可能使用 C++对应头文件而非 C 语言的头文件。

   如，鼓励使用 `#include <ctime>`, `#include <cmath>`, `#include <cstdio>`,
   `#include <cstring>` 的写法。请尽量杜绝使用`#include <time.h>`,
   `#include <math.h>`, `#include <stdio.h>`， `#include <string.h>` 的写法。

1. 只包含必需的头文件。不多，也不少。

   另外，请注意头文件包含顺序。可运行 `apollo.sh format -c` 或
   `scripts/clang_format.sh` 来修复头文件顺序问题。

1. 在 Bazel 目标的`deps`部分，只列出该目标的直接依赖。一般来说，只需要列举出该目
   标包含的头文件所在的 Bazel 目标作为依赖项即可。

   举例，假设`sandwich.h`包含`bread.h`，而`bread.h`又包含`flour.h`。由
   于`sandwich.h`并不直接包含`flour.h` (毕竟，谁会想在三明治中加面粉呢？）
   ，BUILD 文件应写作：

   ```python
   cc_library(
    name = "sandwich",
    srcs = ["sandwich.cc"],
    hdrs = ["sandwich.h"],
    deps = [
        ":bread",
        # BAD practice to uncomment the line below
        # ":flour",
    ],
   )

    cc_library(
        name = "bread",
        srcs = ["bread.cc"],
        hdrs = ["bread.h"],
        deps = [":flour"],
    )

    cc_library(
        name = "flour",
        srcs = ["flour.cc"],
        hdrs = ["flour.h"],
    )
   ```

1. 遵循 DRY（不要重复）的原则。

   避免重复的类，函数，常量定义，尽量避免重复的代码块。举例：

   - 用完整路径引用某个名字，如 `apollo::common::util::Type`，一次是 OK 的。但如
     果要使用两次或者更多次，建议设置一个短别名：
     `using apollo::common::util::Type;`.

   - 用级联的方式访问 Protobuf 中的子字段是 OK 的，如，
     `a_proto.field_1().field_2().field_3()`, 但如果要访问多次，最好将共同前缀部
     分保存为引用： `const auto& field_2 = a_proto.field_1().field_2();`.
