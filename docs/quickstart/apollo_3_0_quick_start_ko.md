# Apollo 3.0 Quick Start Guide

The following guide serves as a user manual for launching the Apollo 3.0
software and hardware stack on vehicle.

The Apollo 3.0 Quick Start Guide focuses on Apollo 3.0's new features. For general
Apollo concepts, please refer to
[Apollo 1.0 Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md).

# Apollo 3.0 빠른 시작 가이드

다음 안내서는 차량에 Apollo3.0 소프트웨어 및 하드웨어들을 시작하기 위한 사용 설명서이다.

Apollo 3.0 빠른 시작 가이드는 Apollo 3.0의 새로운 기능에 중점을 둡니다. 일반적인 Apollo 개념은 [Apollo 1.0 빠른 시작](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md)을 참조하십시오

## Contents

- [Calibration Guide](#calibration-guide)
- [Hardware and Software Installation](#hardware-and-software-installation)
- [Dreamview Usage Table](#dreamview-usage-table)
- [Onboard Test](#onboard-test)

## 콘텐츠

- [조정 가이드](#calibration-guide)
- [하드웨어와 소프트웨어 설치](#hardware-and-software-installation)
- [드림뷰 사용 테이블](#dreamview-usage-table)
- [차내 테스트](#onboard-test)


## Calibration Guide

For the vehicle's onboard testing make sure you have calibrated all the sensors. For
sensor calibration, please refer to
[Apollo 2.0 Sensor Calibration Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide.md)
before you proceed.

## 조정 가이드

차량으로 차내 테스트를 할 때 모든 센서들을 조정하세요. 센서들을 조정하는데에 있어, 진행하기에 앞서 [Apollo 2.0 센서 조정 가이드](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide.md)를 참고하세요.

## Hardware and Software Installation

Please refer to
[Apollo 3.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_hardware_system_installation_guide.md)
for the steps to install the hardware components and the system software, as well as
[Apollo Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md).

## 하드웨어와 소프트웨어 설치

하드웨어 요쇼를 설치하기 위해 [Apollo 3.0 하드웨어와 시스템 설치 가이드](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_hardware_system_installation_guide.md)를 참고하고, 시스템 소프트웨어도 [Apollo 소프트웨어 설치 가이드](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md)를 참고하세요.

## Dreamview Usage Table

For questions regarding Dreamview icons refer to the
[Dreamview Usage Table](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/dreamview_usage_table.md).

## Dreamview 사용 테이블

Dreamview 아이콘에 관한 질문은 Dreamview 사용 표를 참조하세요.

## Onboard Test

1. Plug-in an external hard-drive to any available USB port in the host machine. 

2. Turn on the vehicle, and then the host machine.

3. Launch Docker Release Container.

4. Launch DreamView.

   Note\: Use your favorite browser to access Dreamview web service in your host
   machine browser with URL http://localhost:8888.

   ![](images/dreamview_2_5.png)

5. Select Mode, Vehicle and Map.

   ![](images/dreamview_2_5_setup_profile.png)

   Note\: You'll be required to setup profile before doing anything else. Click
   the dropdown menu to select **Navigation** mode, the HDMap and vehicle you
   want to use. The lists are defined in
   [HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/dreamview/conf/hmi.conf).

   Note\: It's also possible to change the profile on the right panel of the
   HMI, but just remember to click `Reset All` on the top-right corner to
   restart the system.

6. Start the Modules.

   Click the `Setup` button.

   ![](images/dreamview_2_5_setup.png)

   Go to **Module Controller** tab, check if all modules and hardware are ready.
   (Note\: In your offline environment, the hardware modules such as GPS,
   CANBus, Velodyne, Camera and Radar cannot be brought up.)
   (Note\: You may need to drive around a bit to get a good GPS signal.)
   
   ![](images/dreamview_2_5_module_controller.png)

7. Under `Default Routing` select your desired route.

8. Under Tasks click `Start Auto`. (Note: Be cautious when starting the autonomous
   driving, you should now be in autonomous mode.)

   ![](images/dreamview_2_5_start_auto.png)

9. After the autonomous testing is complete, under Tasks click `Reset All`, close all
   windows and shutdown the machine. 

10. Remove the hard drive.

## 차내 테스트

1. 외장형 하드 드라이브를 호스트 컴퓨터의 사용 가능한 USB 포트에 연결하세요.

2. 차량에 시동을 걸고, 기계를 작동시키세요.

3. Docker를 실행 Container 실행.

4. Dreamview 실행.

   참고\: URL http://localhost:8888 이있는 호스트 컴퓨터 브라우저에서 Dreamview 웹 서비스에 액세스하려면 즐겨 사용하는 브라우저를 사용하세요

   ![](images/dreamview_2_5.png)

5. 모드와, 차량과 지도를 선택하세요.

   ![](images/dreamview_2_5_setup_profile.png)
   
   참고\: 다른 작업을 수행하기 전에 프로필을 설정해야 합니다. 메뉴에서 사용하고 싶은 **Navigation** 모드, HD지도와 
   차량을 선택하세요. 리스트들은 [HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/dreamview/conf/hmi.conf)에 정의되어 있습니다.
  
   참고\: HMI의 오른쪽 패널에서 프로필을 변경할 수도 있지만, 시스템을 다시 시작하려면 오른쪽 상단에 있는 `Reset All`을 클릭하세요.

6. 모듈 시작.

   `Setup` 버튼을 누르세요.

   ![](images/dreamview_2_5_setup.png)

    **Mode Controller** 탭으로 이동해서, 모든 모듈과 하드웨어가 준비되어 있는지 확인하세요.
   (참고\: 오프라인 환경에서는 GPS, CANBus, Velodyne, Camera와 Radar와 같은 하드웨어 모듈을 시작할 수 없습니다.) 
   (참고\: 좋은 GPS 신호를 얻으려면 약간의 주행이 필요할 수 있습니다.)

   ![](images/dreamview_2_5_module_controller.png)

7. `Default Routing`에서 당신이 원하는 경로를 선택하세요

8. 할 일 목록 중에 `Start Auto`를 클릭하세요.
   (참고: 자율주행을 시작할 때 조심하세요. 당신은 이제 자율주행 모드입니다.) 

   ![](images/dreamview_2_5_start_auto.png)

9. 자율주행 모드를 끝낸후, 할 일 목록 중에 `Reset All`을 클릭하고, 모든 윈도우와 기계들을 종료하세요.

10. 하드 드라이브를 삭제하세요.
