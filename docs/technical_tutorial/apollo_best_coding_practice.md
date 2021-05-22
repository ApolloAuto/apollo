# Apollo Best Coding Practice

1. Always build, test, and lint all.

   ```bash
   ./apollo.sh check
   ```

1. Always write unit tests and put them along with the source files.

   ```text
   foobar.h
   foobar.cc
   foobar_test.cc
   ```

1. A Bazel target should contain at most one header and one source file.

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

   You can use `./apollo.sh format <path/to/BUILD>` to fix BUILD file style
   issues.

1. In general, Apollo follows
   [Google C++ coding style](https://google.github.io/styleguide/cppguide.html).
   You should run `scripts/clang_format.sh <path/to/cpp/dirs/or/files>` or
   `./apollo.sh format -c <path/to/cpp/dirs/or/files>` to fix C++ style issues.

1. Simple and unified function signature.

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

1. Use const whenever possible.

   ```C++
   // Variables that don't change.
   const size_t current_size = vec.size();
   // Functions that have no side effect.
   const std::string& name() const;
   ```

1. Prefer C++ headers over C headers.

   We prefer using `#include <ctime>` over `#include <time.h>`, `<cmath>` over
   `<math.h>`, `<cstdio>` over `<stdio.h>`, `<cstring>` over `<string.h>`, etc.

1. Include necessary headers **only**. No more, no less.

   Please also pay attention to header orders. Again, you can use
   `apollo.sh format -c` or `scripts/clang_format.sh` to fix header order
   issues.

1. List only direct dependencies in `deps` section of a Bazel target.

   Generally, only targets to which the included headers belongs should be
   listed as a dependency.

   For example, suppose `sandwich.h` includes `bread.h` which in turn includes
   `flour.h`. Since `sandwich.h` doesn't include `flour.h` directly (who wants
   flour in their sandwich?), the BUILD file would look like this:

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

1. Conform to the DRY principle.

   Don't repeat yourself, in any way. Avoid duplicate classes, functions, const
   variables, or a simple piece of code. Some examples:

   - It's fine to refer a name with full path once, like
     `apollo::common::util::Type`, but better to make a short alias if you need
     to use twice or more: `using apollo::common::util::Type;`.
   - It's fine to access a sub-field of proto once in cascade style, like
     `a_proto.field_1().field_2().field_3()`, but better to save the reference
     of a common part first if you need to access it twice or more:
     `const auto& field_2 = a_proto.field_1().field_2();`.
