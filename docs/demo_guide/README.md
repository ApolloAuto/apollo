# 오프라인 데모 실행
 아폴로는 당신이 시뮬레이션에 필요한 하드웨가 없어도 시뮬레이션을 할 수 있는 방법을 제공합니다.
 [Build and Release](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)
페이지의 
[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#docker)
섹션에 있는 지침에 따라 도커 릴리즈 환경을 설정합니다.
 설정 순서:
1. 밑의 명령어를 사용하여 도커 릴리즈 환경을 실행하세요:
    ```
    bash docker/scripts/dev_start.sh
    ```
 2. 도커 릴리즈 환경을 입력하세요:
    ```
    bash docker/scripts/dev_into.sh
    ```
 3. 컨테이너에 아폴로를 구축하세요:
    ```
    bash apollo.sh build
    ```
    `Note:` 당신이 만약 GPU가 없다면, 다음 스크립트를 사용하면 됩니다.
    ```
    bash apollo.sh build_cpu
    ```
4. ros call과 Monitor module과 Dreamview를 시작하기 위한 부트스트랩
    ```
    bash scripts/bootstrap.sh
    ```
 5. 이제 당신은 rosbag을 사용할 수 있습니다:
    ```
    sudo python docs/demo_guide/rosbag_helper.py demo_2.0.bag #download rosbag
    rosbag play demo_2.0.bag --loop
    ```
     `--loop` 옵션은 rosbag이 루프 재생 모드로 사용할 수 있게 해줍니다.
 6. 크롬을 열고 아래 화면에 나와 있는 것처럼 아폴로 드림뷰에 접속하기 위해 **localhost:8888**으로 가세요,.
    ![](images/dv_trajectory.png)
    드림뷰에 있는 자동차는 이제 움직일 수 있습니다.
    
     축하합니다.
