# How to Lint and Format

## Coding style

- **C/C++ coding style**: Apollo adopted the
  [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

## Linters provided in apollo

Apollo uses the following lint tools to check the code style. **Note:** As for
now, ci will only check the style of C++ codes and it may do so in the future
for codes in other languages.

|    tool    |               file type               |          usage (from Apollo root dir)           |
| :--------: | :-----------------------------------: | :---------------------------------------------: |
|  cpplint   | .h .c .hpp .cpp .hh .cc .hxx .cxx .cu | ./apollo.sh lint cpp <path/to/src/dir/or/files> |
|   flake8   |                  .py                  | ./apollo.sh lint py <path/to/src/dir/or/files>  |
| shellcheck |           .sh .bash .bashrc           | ./apollo.sh lint sh <path/to/src/dir/or/files>  |

To make sure your code conforms to the style guide, you can use command
`bash apollo.sh lint all` to check if your code has any style problems and then
fix them manually. Another option is to use formatters to fix style problems
automatically.

## Formatters provided in apollo

Apollo integrated a set of formatting tools that cover many types of files:

|     tool     |               file type               |           usage (from Apollo root dir)           | configuration file |
| :----------: | :-----------------------------------: | :----------------------------------------------: | :----------------: |
| clang-format | .h .c .hpp .cpp .hh .cc .hxx .cxx .cu | ./apollo.sh format -c <path/to/src/dir/or/files> |   .clang-format    |
|   autopep8   |                  .py                  | ./apollo.sh format -p <path/to/src/dir/or/files> |      tox.ini       |
|  buildifier  |       BUILD .BUILD .bzl .bazel        | ./apollo.sh format -b <path/to/src/dir/or/files> |         NA         |
|    shfmt     |           .sh .bash .bashrc           | ./apollo.sh format -s <path/to/src/dir/or/files> |   .editorconfig    |
|   prettier   |            .md .json .yaml            | ./apollo.sh format -m <path/to/src/dir/or/files> |   .prettier.json   |

For ease of use, you can format all the files with types listed above using
command `./apollo.sh format -a <path/to/src/dir/or/files>`
