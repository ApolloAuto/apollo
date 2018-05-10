## How to Contribute to Apollo

### Contributor License Agreements

You are welcome to contribute to project Apollo. To contribute to apollo, you have to agree with the [Apollo individual contributor license agreement](
                https://gist.githubusercontent.com/startcode/f5ccf8887bfc7727a0ae05bf0d601e30/raw/029a11300e987e34a29a9d247ac30caa7f6741a7/Apollo_Individual_Contributor_License_Agreement).

### How to start contribute

You can follow the standard [github approach](https://help.github.com/articles/using-pull-requests/) to contribute code.
There are [issues with label "help wanted"](https://github.com/ApolloAuto/apollo/labels/help%20wanted) that are best to get started.
If you decided to work on an issue, you can leave a message in that issue to let other people know that you are working on it.


Before sending your pull request for
[review](https://github.com/ApolloAuto/apollo/pulls),
make sure your changes follow the coding style, license and testing guidelines.

#### License

For each new file, please include a license at the top of the file.

* C++ code License example [adapter.h](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter.h);

* Python code License example [diagnostics.py](https://github.com/ApolloAuto/apollo/blob/master/modules/tools/diagnostics/diagnostics.py);

* Bash code License example [apollo_base.sh](https://github.com/ApolloAuto/apollo/blob/master/scripts/apollo_base.sh);

#### Testing

Please include unit tests for the contributed code to prove that your code works correctly,
and make sure that your code does not break existing tests. Test files are always ended with `test.cc`, and the test target names in BUILD file are always ended with `test`.
Here is an example test file [adapter_test.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter_test.cc).

You can use command `bash apollo.sh test` to run all unit tests.

#### Coding style

* C/C++ coding style: Apollo adopted the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Make sure your code conforms to this style guide. You can use command `bash apollo.sh lint` to check if your code has any style problem.

* Python coding style:  Apollo adopted the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html). You can use the  [yapf](https://github.com/google/yapf) command `yapf -i --style='{based_on_style: google}' foo.py` to format a file foo.py.

* BUILD coding style : you can use command `bash apollo.sh buildify` to format your BUILD files before submit.

#### Documentation

If your code is not straightforward for other contributors to understand, it is recommended to implement the code in a clear and efficient way, and provide sufficient documentation.
Apollo uses doxygen to help generate formatted API Document with command `bash apollo.sh doc`.
Document your code following this guide [How to document code](docs/howto/how_to_document_code.md).

#### Commit Message
The first line of commit message should be a one-line summary of the change.
A paragraph can be added following the summary to clearly explain the details of the change.
If your code fixed a issue, add the issue number.
The following is a commit message example:


> Replace algorithm A with algorithm B in apollo/modules/control.
>
> Algorithm B is faster than A because it uses binary search. The runtime is reduced from O(N) to O(log(N)).
>
> Fixes #1234

#### Before Creating Pull Request
After you finish your code and ready to create a Pull Request, please make sure your
change doesn't break build/test/lint by running `bash apollo.sh check`, which is
equivalent to a combination of `bash apollo.sh build`, `bash apollo.sh test` and
`bash apollo.sh lint`.
