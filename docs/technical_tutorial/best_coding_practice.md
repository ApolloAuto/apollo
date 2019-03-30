# Best Coding Practice

1. Always compile all, test all and lint all.

   ```bash
   apollo.sh check
   ```

1. Always write unit test and put it along with the source code.

   ```text
   foobar.h
   foobar.cc
   foobar_test.cc
   ```

1. A bazel target contains at most one header and one source file.

   ```text
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

   You could setup and run `scripts/buildifier.sh <some/path>` to fix BUILD file
   style issues.

1. In general, we follow
   [Google C++ coding style](https://google.github.io/styleguide/cppguide.html).
   You could run `scripts/clang-format.sh <some/path>` to fix C++ style issues.

1. Simple and unified function signature.

   ```C++
   // 1. For input objects, const reference guarantes that it is valid, while
   //    pointers might be NULL or wild. Don't give others the chance to break
   //    you.
   // 2. For input scalas, just pass by value, which gives better locality and
   //    thus performance.
   // 3. For output, it's callers' responsibility to make sure the pointers are
   //    valid. No need to do sanity check, and also no need to mark as
   //    "OutputType* const", because pointer redirection is never allowed.
   void FooBar(const InputObjectType& input1, const InputScalaType input2, ...,
               OutputType* output1, ...);

   // RVO machanism will help you avoid unnecessary object copy.
   // See https://en.wikipedia.org/wiki/Copy_elision#Return_value_optimization
   OutputType FooBar(const InputType& input);
   ```

1. Use const whenever possible.

   ```C++
   // Variables that don't change.
   const int current_size = vec.size();
   // Functions that have no side effect.
   const std::string& name() const;
   ```

1. The DRY principle.

   Don't repeat yourself, in any way. Avoid duplicate classes, functions, const
   variables, or a simple piece of code. Some examples:

   - It's fine to refer a name with full path once, like
     `apollo::common::util::Type`, but better to make a short alias
     if you need to use twice or more: `using apollo::common::util::Type;`.
   - It's fine to access a sub-field of proto once in cascade style, like
     `a_proto.field_1().field_2().field_3()`, but better to save the reference
     of a common part first if you need to access it twice or more:
     `const auto& field_2 = a_proto.field_1().field_2();`.
