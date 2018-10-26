## 아폴로에 기여 하는 법

### 기여 허가증 동의서
아폴로 프로젝트에 참여하세요. 아폴로 프로젝트에 기여하기 위해서 당신은 아폴로 프로젝트 라이선스에 동의해야 합니다.[Apollo individual contributor license agreement](https://gist.githubusercontent.com/startcode/f5ccf8887bfc7727a0ae05bf0d601e30/raw/029a11300e987e34a29a9d247ac30caa7f6741a7/Apollo_Individual_Contributor_License_Agreement).

### 기여 하는 방법
*기여하기 위해서 당신은 Github표준을 따라야 합니다.[Github approach](https://help.github.com/articles/using-pull-requests/)
* "help wanted"라고 하는 안내서가 당신이 시작하기에 도움을 줄 것입니다.[issues with label "help wanted"](https://github.com/ApolloAuto/apollo/labels/help%20wanted)
* 만약 당신이 어떤 문제에 관해 작업을 하고 있다면, 당신이 무슨 작업을 하고 있는지 사람들이 알 수 있게 메세지를 남기세요.
[review](https://github.com/ApolloAuto/apollo/pulls),
pull request를 보내기 이전에, 안내서와 변경 사항이 라이센스, 테스팅, 코딩 스타일 지침이 아래에 언급된 지침을 따르는지 확인하세요.


#### License
#### 라이선스
For each new file, please include a license at the top of the file.
각각의 새로운 파일들에 파일의 제일 위에 있는 라이선스를 포함하여 주십시오.
*  C++ 코드 라이선스 예시 [adapter.h](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter.h);
* Python 코드 라이선스 예시 [diagnostics.py](https://github.com/ApolloAuto/apollo/blob/master/modules/tools/diagnostics/diagnostics.py);
* Bash 코드 라이선스 예시 [apollo_base.sh](https://github.com/ApolloAuto/apollo/blob/master/scripts/apollo_base.sh);


#### 테스팅
당신의 코드가 올바르게 작동한다는 것을 보여주기 위해 코드에 유닛 테스트를 포함하고, 당신의 코드가 실행 중에 올바르게 작동하지 않는지 확인하세요.

실행 코드의 마지막은 항상 `_test.cc`로 끝나야 하며, BUILD파일에 있는 테스트 대상 이름은 항상 `test`로 끝나야 합니다.
다음은 [adapter_test.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/common/adapters/adapter_test.cc);
파일의 예시입니다.

당신은 유닛 테스트를 실행시키기 위해 `bash apollo.sh test`명령어를 사용할 수 있습니다.

#### 코딩 스타일

* **/C++ 코딩 스타일: Apollo는 Google c++ 스타일 가이드를 채택했습니다.[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
	당신의 코드가 이 가이드와 부합되는지 확인하세요.
	당신의 코드 스타일이 문제가 있는지 확인하기 위해 `bash apollo.sh lint`명령어를 사용할 수 있습니다.

* **Python 코딩 스타일: Apollo는 Google Python 스타일 가이드를 채택했습니다. [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
	당신은 foo.py 파일을 포맷하기 위해 [yapf](https://github.com/google/yapf)를 이용하여  `yapf -i --style='{based_on_style: google}' foo.py` 명령어를 사용할 수 있습니다.

* **BUILD 코딩 스타일: 당신은 제출하기 전에 BUILD 파일을 포맷하기 위해 `bash apollo.sh buildify` 명령어를 사용할 수 있습니다.

#### 문서화
만약 당신의 코드가 다른 기여자들이 이해하기에 쉽지 않다면, 코드를 명확하고 효율적으로 구현할 것을 권장합니다.
그리고 충분한 설명과 문서화를 제공하기를 바랍니다. Apollo는 명령을 사용하여 포맷된 API문서를 생성할 수 있도록 지원하기 위해 `bash apollo.sh doc`명령어를 사용합니다.
[How to document code](docs/howto/how_to_document_code.md).가이드에 따라서 코드를 문서화하세요.

#### 커밋 메시지

커밋 메시지의 첫 번째 줄은 변경된 것들에 대한 간략한 정리입니다.
정리된 것에 따라 변경 내용을 명확히 설명하는 단락을 추가할 수 있습니다.
만약 당신의 코드가 문제를 해결한 경우, 커밋 메시지에 번호를 추가하세요.
다음은 커밋 메시지의 좋은 예시입니다.
> apollo/modules/control에 있는 알고리즘 A를 B로 대체하였습니다.
> 바이더니 탐색을 사용하기 때문에 알고리즘 B가 A보다 빠릅니다. 런타임 시간을 O(N)에서 O(log(N))으로 줄여 주었습니다.
> Fixes #1234
### Pull Request를 생성하기 전에
코드 작업을 마친 후 Pull Request를 생성할 준비가 되었을 때, `bash apollo.sh check`를 실행하는 도중 당신의 코드가 build/test/lint를 중단하지 않게 해주십시오, 그것은 `bash apollo.sh build`, `bash apollo.sh test` 그리고 `bash apollo.sh lint`의 조합과 같습니다.

