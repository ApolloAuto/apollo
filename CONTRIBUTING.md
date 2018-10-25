## How to Contribute to Apollo
## 아폴로에 기여 하는 법

### Contributor License Agreements
### 기여 허가증 동의서

You are welcome to contribute to project Apollo. To contribute to apollo, you have to agree with the [Apollo individual contributor license agreement](
아폴로 프로젝트에 참여하세요. 아폴로 프로젝트에 기여하기 위해서 당신은 아폴로 프로젝트 라이선스에 동의해야 합니다.

                https://gist.githubusercontent.com/startcode/f5ccf8887bfc7727a0ae05bf0d601e30/raw/029a11300e987e34a29a9d247ac30caa7f6741a7/Apollo_Individual_Contributor_License_Agreement).

### How do I start contributing
### 어떻게 기여를 해야 하는가.

* You can follow the standard [Github approach](https://help.github.com/articles/using-pull-requests/) to contribute code.
* 기여하기 위해서 당신은 Github표준을 따라야 합니다.

* There are [issues with label "help wanted"](https://github.com/ApolloAuto/apollo/labels/help%20wanted) that are best to help you get started.
* "help wanted"라고 하는 안내서가 당신이 시작하기에 도움을 줄 것입니다.

* If you are currently working on an issue, leave a message to let people know that you are working on it.
* 만약 당신이 어떤 문제에 관해 작업을 하고 있다면, 당신이 무슨 작업을 하고 있는지 사람들이 알 수 있게 메세지를 남기세요.

* Before sending in your pull request for

[review](https://github.com/ApolloAuto/apollo/pulls),
make sure your changes follow the guidelines mentioned below, namely: license, testing and coding style guidelines.
* pull request를 보내기 이전에, 안내서와 변경 사항이 라이센스, 테스팅, 코딩 스타일 지침이 아래에 언급된 지침을 따르는지 확인하세요.

#### License
#### 라이선스

For each new file, please include a license at the top of the file.
각각의 새로운 파일들에 파일의 제일 위에 있는 라이선스를 포함하여 주십시오

* C++ code License example [adapter.h](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter.h);
* C++ 코드 라이선스 예시 adapter.h;

* Python code License example [diagnostics.py](https://github.com/ApolloAuto/apollo/blob/master/modules/tools/diagnostics/diagnostics.py);
* Python 코드 라이선스 예시 diagnostics.py;

* Bash code License example [apollo_base.sh](https://github.com/ApolloAuto/apollo/blob/master/scripts/apollo_base.sh);
* Bash 코드 라이선스 예시 apollo_base.sh;

#### Testing
#### 테스팅

Please include unit tests for the contributed code to prove that your code works correctly,
and make sure that your code does not break existing tests. Test files are always named to end with `_test.cc`, and the test target names in the BUILD file always end with `test`.
당신의 코드가 올바르게 작동한다는 것을 보여주기 위해 코드에 유닛 테스트를 포함하고, 당신의 코드가 실행 중에 올바르게 작동하지 않는지 확인하세요
실행 코드의 마지막은 항상 _test.cc로 끝나야 하며, BUILD파일에 있는 테스트 대상 이름은 항상 test로 끝나야 한다.

Here is an example test file [adapter_test.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter_test.cc).
다음은 adapter_test.cc파일의 예시이다.

You can use command `bash apollo.sh test` to run all unit tests.
당신은 모든 유닛 테스트를 실행시키기 위해 'bash apollo.sh test'명령어를 사용할 수 있습니다.

#### Coding style
#### 코딩 스타일

* **C/C++ coding style**: Apollo adopted the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Make sure your code conforms to this style guide. You can use command `bash apollo.sh lint` to check if your code has any style problem.
* **C/C++ 코딩 스타일: Apollo는 Google c++ 스타일 가이드를 채택했습니다. 당신의 코드가 이 가이드와 부합되는지 확인하세요.
	당신의 코드 스타일이 문제가 있는지 확인하기 위해 bash apollo.sh lint명령어를 사용할 수 있습니다.

* **Python coding style**:  Apollo adopted the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html). You can use the  [yapf](https://github.com/google/yapf) command `yapf -i --style='{based_on_style: google}' foo.py` to format a file foo.py.
* **Python 코딩 스타일: Apollo는 Google Python 스타일 가이드를 채택했습니다.
	당신은 foo.py 파일을 포맷하기 위해 yapf -i --style='{based_on_style: google}' foo.py 명령어를 사용할 수 있습니다.

* **BUILD coding style** : you can use command `bash apollo.sh buildify` to format your BUILD files before you submit.
* **BUILD 코딩 스타일: 당신은 제출하기 전에 BUILD 파일을 포맷하기 위해 bash apollo.sh buildify 명령어를 사용할 수 있습니다.

#### Documentation
#### 문서화

If your code is not straightforward for other contributors to understand, it is recommended to implement the code in a clear and efficient way, and provide sufficient comments and documentation.
만약 당신의 코드가 다른 기여자들이 이해하기에 쉽지 않다면, 코드를 명확하고 효율적으로 구현할 것을 권장합니다.
그리고 충분한 설명과 문서화를 제공하기를 바랍니다.

Apollo uses doxygen to help generate formatted API Document with command `bash apollo.sh doc`.
Document your code following this guide [How to document code](docs/howto/how_to_document_code.md).
 Apollo는 명령을 사용하여 포맷된 API문서를 생성할 수 있도록 지원하기 위해 bash apollo.sh.doc명령어를 사용합니다.
How to document code 가이드에 따라서 코드를 문서화하세요.


#### Commit Message
#### 커밋 메시지

The first line of commit message should be a one-line summary of the change.
commit message의 첫 번째 줄은 변화된 것들에 대한 간략한 정리입니다.
A paragraph can be added following the summary to clearly explain the details of the change.
정리된 것에 따라 변경 내용을 명확히 설명하는 단락을 추가할 수 있습니다.
If your code fixed an issue, add the issue number to your commit message.
약 당신의 코드가 문제를 해결한 경우, commit message에 번호를 추가하여라.
An example of a good commit message is:
 다음은 commit message의 좋은 예시입니다.
> Replaced algorithm A with algorithm B in apollo/modules/control.
>apollo/modules/control에 있는 알고리즘 A를 B로 대체하였다.

> Algorithm B is faster than A because it uses binary search. The runtime is reduced from O(N) to O(log(N)).
>바이더니 탐색을 사용하기 때문에 알고리즘 B가 A보다 빠르다. 런다임 시간을 O(N)에서 O(log(N))으로 줄여 주었다.

> Fixes #1234
> Fixes #1234

### Before Creating Pull Request
### Pull Request를 생성하기 전에
After you finish your code and ready to create a Pull Request, please make sure your
change don't break build/test/lint by running `bash apollo.sh check`, which is
equivalent to a combination of `bash apollo.sh build`, `bash apollo.sh test` and
`bash apollo.sh lint`.
코드 작업을 마친 후 Pull Request를 생성할 준비가 되었을 때, bash apollo.sh check를 실행하는 도중 당신의 코드가 build/test/lint를 중단하지 않게 해주십시오. 그것은 bash apollo.sh build, bashapollo.sh test 그리고 bash apollo.sh lint의 조합과 같습니다.

