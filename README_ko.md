![image alt text](docs/demo_guide/images/Apollo_logo.png)

[![Build Status](https://travis-ci.com/ApolloAuto/apollo.svg?branch=master)](https://travis-ci.com/ApolloAuto/apollo) [![Simulation Status](https://azure.apollo.auto/dailybuildstatus.svg)](https://azure.apollo.auto/dailybuild)

```

We choose to go to the moon in this decade and do the other things,

not because they are easy, but because they are hard.

-- John F. Kennedy, 1962

```
```

우리는 10년 안에 달에 사람을 보내기로 했습니다. 
이 일은 쉽기 때문이 아니라, 어렵기 때문에 하는 것입니다.

-- 존 F 케네디 대통령, 1962

```

Welcome to Apollo's GitHub page!

[Apollo](http://apollo.auto) is a high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.

For business and partnership, please visit [our website](http://apollo.auto).

Apollo's GitHub 페이지에 온 것을 환영합니다!

[Apollo](http://apollo.auto)는 자율주행 자동차의 개발, 테스트 및 배포를 가속화하는 고성능의 유연한 구조/구성입니다.

비즈니스 및 파트너 관계를 보려면 다음 사이트를 방문하십시오. [우리의 웹사이트](http://apollo.auto).


## Table of Contents

1. [Getting Started](#getting-started)
2. [Prerequisites](#prerequisites)
    - [Basic Requirements](#basic-requirements)
    - [Individual Version Requirements](#individual-version-requirements)
3. [Architecture](#architecture)
4. [Installation](#installation)
5. [Documents](#documents)

## 콘텐츠

1. [시작/프로그램 착수](#getting-started)
2. [전제 조건](#prerequisites)
    - [기본 조건](#basic-requirements)
    - [개인 버전 조건](#individual-version-requirements)
3. [구조](#architecture)
4. [설치](#installation)
5. [문서](#documents)


## Getting Started

**The Apollo Team now proudly presents to you the latest [version 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_quick_start.md).**

## 시작/프로그램 착수

**apollo팀은 지금 가장 최신의 버전 3.0을 공개하였다. [version 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_quick_start.md).**

 Apollo 3.0 is loaded with new modules and features, but needs to be calibrated and configured perfectly before you take it for a spin. Please review the prerequisites and installation steps in detail to ensure that you are well equipped to build and launch Apollo. You could also check out Apollo's architecture overview for a greater understanding on Apollo's core technology and platform. 

apollo 3.0은 새로운 모듈과 기능이 탑재되어 있지만 동작하기 전에 보정 및 구성이 완벽해야한다. 전제조건과 설치 과정을 자세히 검토하여 Apollo를 빌드하고 실행하기 위해 잘 갖추어 젔는지 확인해야한다. 너는 또한  Apollo의 핵심 기술과 플랫폼에 대한 더 깊은 이해를 위해 Apollo의 구조의 개요를 확인할 수도 있다. 

[Want to contribute to our code?](https://github.com/ApolloAuto/apollo/blob/master/CONTRIBUTING.md) follow this guide.

[우리의 코드에 기여하고 싶나?](https://github.com/ApolloAuto/apollo/blob/master/CONTRIBUTING.md) 그럼 우리의 가이드를 따라라.

## Prerequisites

## 전제 조건

#### Basic Requirements:

* Vehicle equipped with by-wire system, including but not limited to brake by-wire, steering by-wire, throttle by-wire and shift by-wire (Apollo is currently tested on Lincoln MKZ)

* A machine with a 4-core processor and 6GB memory minimum

* Ubuntu 14.04

* Working knowledge of Docker

#### 기본 조건:

* 브레이크 바이 와이어, 스티어링 바이 와이어, 스로틀 바이 와이어 및 시프트 바이 와이어를 포함하여 바이 와이어 시스템이 장착 된 차량 (Apollo는 현재 링컨 MKZ에서 테스트 됨)

* 4코어 프로세서 및 최소 6GB 메모리가 장착 된 컴퓨터

* 우분투 14.04

* 도커(소프트웨어 컨테이너 안에 응용 프로그램들을 배치시키는 일을 자동화해 주는 소프트웨어)에 대한 실무 지식



 - Please note, it is recommended that you install the versions of Apollo in the following order: 
 **1.0 > 1.5 > 2.0 > 2.5 > 3.0**.
 The reason behind this recommendation is that you need to confirm whether individual hardware components 
 and modules are functioning correctly and clear various version test cases,
 before progressing to a higher more capable version for your safety and the safety of those around you.

 - Please note, if you do not have a vehicle, proceed to the [Installation - Without Hardware](#without-hardware) 

- 다음 순서로 Apollo 버전을 설치하는 것이 좋습니다: 
 **1.0 > 1.5 > 2.0 > 2.5 > 3.0**.
 이 권장사항의 이유는 개별 하드웨어 구성 요소와 모듈이 올바르게 작동하는지 여부를 확인하고 다양한 버전 테스트 케이스를 지우고 안전성과 주변 사람들의 안전을 위해 더 높은 성능의 버전으로 진행해야하기 때문이다.

 - 만약 차가 없다면, [하드웨어 없이 설치](#without-hardware) 진행하세요 . 


#### Individual Version Requirements:

#### 개인 버전 조건:

The following diagram highlights the scope and features of each Apollo release:

이 표의 하이라이트된 부분은 각 Apollo 배포의 범위와 특징을 나타낸다:

![](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/apollo_versions_3.png)

[**Apollo 1.0:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md) 

Apollo 1.0 also referred to as the Automatic GPS Waypoint Following, works in an enclosed venue such as a test track or parking lot. This installation is necessary to ensure that Apollo works perfectly with your vehicle. The diagram below lists the various modules in Apollo 1.0.

[**Apollo 1.0:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md) 

Apollo 1.0은 자동 GPS 웨이 포인트 추적이라고도 하며 테스트 트랙이나 추차장과 같은 밀폐된 장소에서 작동한다.
이 설치는 Apollo가 차량과 완벽하게 작동하도록 하기 위해 필요합니다. 아래 표는 Apollo1.0의 다양한 모듈을 나열합니다.


![image alt text](docs/demo_guide/images/Apollo_1.png)

**For Setup:**

**설치를 위해:**

* **Hardware**:

    * Industrial PC (IPC)

    * Global Positioning System (GPS)

    * Inertial Measurement Unit (IMU)

    * Controller Area Network (CAN) card

    * Hard drive

    * GPS Antenna

    * GPS Receiver
    

* **하드웨어**:

    * 산엽용 pc(IPC)

    * 위성 항법 시스템(GPS)

    * 관성측정 장치(IMU)

    * 계측 제어기 통신망 카드(CAN card)

    * 하드 드라이브

    * GPS 안테나

    * GPS 수용자



* **Software**:

    * Apollo Linux Kernel (based on Linux Kernel 4.4.32)
    

* **소프트웨어**:

    * Apollo 리눅스 커널(리눅스 커널 4.4.32 기반)


[**Apollo 1.5:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_hardware_system_installation_guide.md) 

Apollo 1.5 is meant for fixed lane cruising. With the addition of LiDAR, vehicles with this version now have better perception of its surroundings and can better map its current position and plan its trajectory for safer maneuvering on its lane. Please note, the modules highlighted in Yellow are additions or upgrades for version 1.5.

Apollo 1,5는 고정 차선 순항을 의미한다. LiDAR이 추가됨에 따라 이 버전의 차량은 주변 환경에 대한 인식이 향상되었으며 현재 위치를 더 잘 파악하고 차선에서 안전한 기동을 위한 궤적을 계획할수 있게 된다. 노란색으로 강조 표시된 모듈은 1.5 버전의 추가.업그레이드 된 사항들이다.

![image alt text](docs/demo_guide/images/Apollo_1_5.png)	

**For Setup:**

* All the requirements mentioned in version 1.0

**설치를 위해:**

* 1.0버전에서 언급한 모든 요구 사항

* **Hardware**:

    * Light Detection and Ranging System (LiDAR)

    * ASUS GTX1080 GPU-A8G- Gaming GPU Card

* **하드웨어**:

    * 항공 레이저측량(LiDAR)

    * ASUS GTX1080 GPU-A8G- Gaming GPU 카드
    
* **Software**:

    * Nvidia GPU Driver
        
* **소프트웨어**:

    * Nvidia GPU 드라이버

[**Apollo 2.0:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md#key-hardware-components)

Apollo 2.0 supports vehicles autonomously driving on simple urban roads. Vehicles are able to cruise on roads safely, avoid collisions with obstacles, stop at traffic lights and change lanes if needed to reach their destination.  Please note, the modules highlighted in Red are additions or upgrades for version 2.0.

Apollo 2.0은 단순한 도시 도로에서 자율 주행하는 차량을 지원한다. 차량은 도로에서 안전하게 크루즈를 할 수 있고 장애물과의 충돌을 피할 수 있으며 신호등에서 멈추고 목적지에 도달해야하는 경우 차선을 변경할 수 있다. 빨간색으로 강조 표시된 모듈은 버전 2.0의 추가 또는 업그레이드입니다.


![image alt text](docs/demo_guide/images/Apollo_2.png)

**For Setup:**

* All the requirements mentioned in versions 1.5 and 1.0

**설치를 위해:**

* 버전 1.5와 1.0에서 언급한 모든 요구사항

* **Hardware**:

    * Traffic Light Detection using Camera

    * Ranging System (LiDAR)

    * Radar
    
* **하드웨어**:

    * 카메라를 이용한 교통 빛 탐지

    * 측정기 시스템(LiDAR)

    * 레이더(Radar)

* **Software**:

    * Same as 1.5
    
* **소프트웨어**:

    * 버전 1.5와 같다

[**Apollo 2.5:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)

Apollo 2.5 allows the vehicle to autonomously run on geo-fenced highways with a camera for obstacle detection. Vehicles are able to maintain lane control, cruise and avoid collisions with vehicles ahead of them. 

Apollo 2.5는 지오펜스가 내에서 고속도로에서 장애물 감지 카메라로 자율적으로 주행 할 수 있다. 차량은 차선 제어를 유지하고 나아갈 수 있으며 앞선 차량과의 충돌을 피할 수 있다.

```
Please note, if you need to test Apollo 2.5; for safety purposes, please seek the help of the
Apollo Engineering team. Your safety is our #1 priority,
and we want to ensure Apollo 2.5 was integrated correctly with your vehicle before you hit the road. 
```

```
Apollo 2.5;를 테스트해야하는 경우 안전을 위해 Apollo 엔지니어링 팀에게 도움을 구하세요. 귀하의 안전은 우리의 #1 최우선 순위이며,
우리는 당신이 길을 가다 사고나기 전에 Apollo 2.5가 차량과 올바르게 통합되도록 보장하고 싶습니다.
```

![image alt text](docs/demo_guide/images/Apollo_2_5.png)

**For Setup:**

* All the requirements mentioned in 2.0

**설치를 위해:**

* 버전 2.0에서 언급한 모든 요구사항

* Hardware:

    * Additional Camera
    
* 하드웨어:

    * 추가적인 카메라

* Software: 

    * Same as 2.0
    
* 소프트웨어: 

    * 버전 2.0과 같다

[**Apollo 3.0:**](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_quick_start.md)

Apollo 3.0's main focus is to provide a platform for developers to build upon in a closed venue low-speed environment. Vehicles are able to maintain lane control, cruise and avoid collisions with vehicles ahead of them. 

Apollo 3.0의 주요 초점은 개발자가 폐쇄 된 장소에서 저속 환경을 구축 할 수있는 플랫폼을 제공하는 것이다. 차량은 차선 제어를 유지하고 진행할 수 있으며 앞선 차량과의 충돌을 피할 수 있다.

![image alt text](docs/demo_guide/images/Apollo_3.0_diagram.png)

**For Setup:**

**설치를 위해:**

* Hardware:

    * Ultrasonic sensors
    * Apollo Sensor Unit
    * Apollo Hardware Development Platform with additional sensor support and flexibility
    
* 하드웨어:

    * 초음파 센서
    * Apollo 센서 단위
    * 추가 센서 지원 및 유연성이있는 Apollo 하드웨어 개발 플랫폼

* Software: 

    * Guardian
    * Monitor
    * Additional drivers to support Hardware
    
* 소프트웨어: 

    * 보호자
    * 모니터
    * 하드웨어를 지원하기 위한 추가적인 드라이버

## Architecture

## 구조

* **Hardware/ Vehicle Overview**

* **하드웨어/ 차량 개요**

![image alt text](docs/demo_guide/images/Hardware_overview.png)

* **Hardware Connection Overview**

* **하드웨어 간의 연결 관계 개요**

![image alt text](docs/demo_guide/images/Hardware_connection.png)

* **Software Overview - Navigation Mode**

* **소프트웨어 개요 - 항법(Navigation) 모드**

![image alt text](docs/specs/images/Apollo_3.0_SW.png)

## Installation

* [Fork and then Clone Apollo's GitHub code](https://github.com/ApolloAuto/apollo) 

* [Build and Release using Docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md) - This step is required

* [Launch and Run Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_launch_Apollo.md)

If at this point, you do not have a Hardware setup, please go to [Without Hardware](#without-hardware). 

## 설치

* [Fork를 한 후 Apollo의 GitHub 코드 복제](https://github.com/ApolloAuto/apollo) 

* [Docker를 사용하여 빌드 및 릴리스](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md) - 이 단계가 요구된다.

* [Apollo를 실행](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_launch_Apollo.md)
 
여기서, 하드웨어 설치가 안되어 있다면, [하드웨어 없이](#without-hardware)로 가세요.

### With Hardware:

* [Apollo 1.0 QuickStart Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md)

* [Apollo 1.5 QuickStart Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_quick_start.md)

* [Apollo 2.0 QuickStart Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_quick_start.md)

* [Apollo 2.5 QuickStart Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_quick_start.md)

* [Apollo 3.0 QuickStart Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_quick_start.md)

### 하드웨어와 함께:

* [Apollo 1.0 빠른시작 안내](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md)

* [Apollo 1.5 빠른시작 안내](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_quick_start.md)

* [Apollo 2.0 빠른시작 안내](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_quick_start.md)

* [Apollo 2.5 빠른시작 안내](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_quick_start.md)

* [Apollo 3.0 빠른시작 안내](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_quick_start.md)

### Without Hardware:

* [How to Build Apollo ](https://github.com/ApolloAuto/apollo/tree/master/docs/demo_guide)

### 하드웨어 없이:

* [어떻게 Apollo를 빌드하는지 ](https://github.com/ApolloAuto/apollo/tree/master/docs/demo_guide)

## Documents

* [Technical Tutorial](https://github.com/ApolloAuto/apollo/tree/master/docs/technical_tutorial): Everything you need to know about Apollo. Written as individual versions with links to every document related to that version.

* [HowTo](https://github.com/ApolloAuto/apollo/tree/master/docs/howto): Brief technical solutions to common problems that developers face during the installation and use of the Apollo platform 

* [Specs](https://github.com/ApolloAuto/apollo/tree/master/docs/specs): A Deep dive into Apollo's Hardware and Software specifications (only recommended for expert level developers that have successfully installed and launched Apollo) 

* [FAQs](https://github.com/ApolloAuto/apollo/tree/master/docs/FAQs) 

## 문서들

* [Technical Tutorial](https://github.com/ApolloAuto/apollo/tree/master/docs/technical_tutorial): 아폴로에 관해 알아야 할 모든 것. 해당 버전과 관련된 모든 문서에 대한 링크가 있는 개별 버전으로 작성됩니다.

* [HowTo](https://github.com/ApolloAuto/apollo/tree/master/docs/howto): 개발자가 Apollo 플랫폼을 설치하고 사용할 때 겪게되는 일반적인 문제에 대한 간략한 기술적 해결책

* [Specs](https://github.com/ApolloAuto/apollo/tree/master/docs/specs): Apollo의 하드웨어 및 소프트웨어 사양에 대해 자세히 설명합니다 (Apollo를 성공적으로 설치하고 시작한 전문가 수준의 개발자에게만 권장)

* [FAQs](https://github.com/ApolloAuto/apollo/tree/master/docs/FAQs) 

## Questions

You are welcome to submit questions and bug reports as [GitHub Issues](https://github.com/ApolloAuto/apollo/issues).

## 질문

질문 및 버그 보고서를 다음 주소로 제출하면 감사하겠습니다. [GitHub Issues](https://github.com/ApolloAuto/apollo/issues).

## Copyright and License

Apollo is provided under the [Apache-2.0 license](https://github.com/natashadsouza/apollo/blob/master/LICENSE).

## 저작권과 라이선스

Apollo는 [Apache-2.0 라이선스](https://github.com/natashadsouza/apollo/blob/master/LICENSE)를 제공합니다.

## Disclaimer

Please refer the Disclaimer of Apollo in [Apollo's official website](http://apollo.auto/docs/disclaimer.html).

## 면책

[Apollo 공식 웹 사이트]에서 Apollo의 면책 조항을 참조하십시오.(http://apollo.auto/docs/disclaimer.html).

## Connect with us 
* [Have suggestions for our GitHub page?](https://github.com/ApolloAuto/apollo/issues)
* [Twitter](https://twitter.com/apolloplatform)
* [YouTube](https://www.youtube.com/channel/UC8wR_NX_NShUTSSqIaEUY9Q)
* [Blog](https://www.medium.com/apollo-auto)
* [Newsletter](http://eepurl.com/c-mLSz)
* Interested in our turnKey solutions or partnering with us Mail us at: apollopartner@baidu.com

## 연락 방법 
* [우리의 GitHub 페이지에 의견이 있습니까?](https://github.com/ApolloAuto/apollo/issues)
* [Twitter](https://twitter.com/apolloplatform)
* [YouTube](https://www.youtube.com/channel/UC8wR_NX_NShUTSSqIaEUY9Q)
* [Blog](https://www.medium.com/apollo-auto)
* [Newsletter](http://eepurl.com/c-mLSz)
* turnKey 문제에 관심이 있거나 우리와 파트너 관계를 맺고 싶다면: apollopartner@baidu.com 으로 메일주세요
