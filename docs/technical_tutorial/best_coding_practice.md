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
1. In general, we follow Google C++ coding style.
   ```https://google.github.io/styleguide/cppguide.html```
   
1. Simple and unified function signature.
   ```C++
   void FooBar(const InputType& input1, const int input2, ...,
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
