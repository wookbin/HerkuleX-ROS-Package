# HerkuleX ROS Package
* Ubuntu 16.04 _ kinetic ver
* HerkuleX ROS Package List
	- HerkuleX (HerkuleX_node, HerkuleX_PanTilt_node, HerkuleX_6DOF_Arm_node, HerkuleX_feedback_Test_node, HerkuleX_12DOF_Robot_node(X))
	- HerkuleX_description (HerkuleX_Arm_Control_moveit.launch, HerkuleX_Arm_display.launch, HerkuleX_Arm_Manual_Control.launch, HerkuleX_pantilt_Control.launch, HerkuleX_pantilt_display.launch)

===[Device Setting]====================================

-	HerkuleX와 PC간의 통신 연결을 위한 시리얼 인터페이스 장치와 전원공급 장치를 준비하여 아래 그림과 같이 연결합니다.

 ![0](https://user-images.githubusercontent.com/58063370/73432722-addf6980-4386-11ea-8a9b-e4104ebc70d8.png)
 
그림 1. HerkuleX와 PC간의 연결 설명서

	요즘 PC에는 대부분 Serial Port가 없기 때문에 위의 그림과 같이 USB to Serial Device를 추가로 사용해야 합니다.

	이 USB to Serial Device를 이용하여, PC와 HerkuleX간의 RS232통신을 하기 때문에 Ubuntu환경에서는 ttyUSB권한 설정 및 USB rule설정을 초기에 1회 해주어야 하며, 그 방법은 다음과 같습니다.

1)	ttyUSB권한 설정.

HerkuleX의 USB Port에 대한 권한은 ‘sudo chmod 777 ttyUSB0’ 명령어를 통해서 줄 수 있지만, 매번 설정이 번거로우므로 dialout그룹에 추가하는 방법을 이용한다.

	sudo usermod -a -G dialout $USER

 * 위의 명령어를 실행 후 PC의 재 시작을 해야 적용된다.
 
2)	ttyUSB rule의 설정을 위한 심볼릭 링크 ttyUSBx만들기

-	심볼릭 링크를 만드는데 필요한 정보는 아래 3가지 이다.

	Vender ID

	Product ID

	Serial Number

-	위의 3가지 정보는 아래 2개의 명령어로 알아낼 수가 있다.

	$ lsusb

![1](https://user-images.githubusercontent.com/58063370/73432967-2b0ade80-4387-11ea-9acc-6bad239121a9.png)

그림 2. USB list

	$ udevadm info -a /dev/ttyUSB0 | grep '{serial}'
 
![001](https://user-images.githubusercontent.com/58063370/73433144-85a43a80-4387-11ea-80a1-65c0a1facccb.png)
 
그림 3. USB to Serial Device Serial Number

위에 출력된 정보가 USB to Serial Device의 Serial Number이며, 해당 번호는 제품별로 상이하다

-	알아낸 정보를 이용한 .rules file생성.

![3](https://user-images.githubusercontent.com/58063370/73433039-4ece2480-4387-11ea-8d70-babf577869ce.png)

그림 4. USB rules file

‘HerkuleX.rules’ 파일을 ‘/etc/udev/rules.d’ 경로에 생성한 후에 해당 파일의 내용을 아래와 같이 작성한 후 저장한다.

KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AK05VTK7", MODE:="0666", GROUP:="dialout", SYMLINK+="HerkuleX"

저장완료 후에는 아래와 같은 명령어를 이용하여 작성 내용의 확인이 가능하다.

![2](https://user-images.githubusercontent.com/58063370/73433002-36f6a080-4387-11ea-9cd2-41d500dbb968.png)
 
그림 5. HerkuleX.rules file확인

-	udev 재시작 명령어를 호출한 후에 PC의 재 시작을 해주어야 적용이 된다.

	$ sudo service udev restart

-	심볼릭 등록의 확인. (아래와 같은 명령어를 이용하여, 심볼릭 링크로 설정한 ttyUSB장치가 tetra로 적용되었는지 여부를 확인한다.)

	$ ll /dev/
 
![4](https://user-images.githubusercontent.com/58063370/73433186-9b196480-4387-11ea-9929-d0e0f0a623e8.png)
 
그림 6. Device List확인




===[HerkuleX Node Start]====================================

ROS시스템에서는 실행하고자 하는 ROS Node를 구동하기 위해서는 초기에 ROS마스터를 실행해 주어야 각각의 Node사이에 연결과 메시지의 통신이 가능하다. HerkuleX Package에서는 이러한 번거로움을 해결하고자 Launch파일을 추가로 제작하였으며, 이 Launch만을 실행 함으로서 HerkuleX의 구동을 할 수 있도록 되어 있다. 해당 Launch file의 구성은 다음과 같다.

![1](https://user-images.githubusercontent.com/58063370/73435004-23e5cf80-438b-11ea-902a-7d931e646090.png)
 
그림 7. HerkuleX Launch file의 구성

1)	pkg: 패키지의 이름.

2)	type: 실제 실행할 Node의 이름.

3)	name: 위 type에 해당하는 Node가 실행될 때 붙여지는 이름.

4)	args: 연결된 HerkuleX의 개수 (Total Axis)

5)	output: 해당 터미널 창에 실행되는 Node들의 출력들 표시해주기 위한 옵션.

※ HerkuleX Node를 실행하기 위해서는 위의 그림과 같이 Launch file을 실행해도 되지만, roscore를 실행한 후에 HerkuleX_Node를 실행하는 방법으로도 사용이 가능하다.

 - HerkuleX를 실행하기 위한 Launch File의 실행은 터미널 창을 열어서 해당 Launch file을 호출하는 명령어를 입력하는 방식으로 진행하며, 그 방법은 다음과 같다.

![1](https://user-images.githubusercontent.com/58063370/73435078-4aa40600-438b-11ea-8554-22b85aeb788f.png)

그림 8. HerkuleX Launch file의 실행을 위한 터미널 창 명령어 입력

	명령어 입력:  $ roslaunch HerkuleX HerkuleX.launch 

- HerkuleX Launch file이 실행되면, 아래 그림과 같이 터미널 창에 HerkuleX의 동작을 위한 입력 창이 출력된다. 이때, HerkuleX와 PC간에 Serial통신이 정상적으로 수행되면 터미널창의 가장 하단에 ‘Serial Port initialized’라는 문구가 나오게 되는데 해당 문구가 출력되지 않으면, PC와 HerkuleX간에 연결이 정상적으로 되지 않는 것이다. (해당 경우에는 HerkuleX를 연결하는 USB to Serial의 연결 및 PC에서의 인식여부, HerkuleX에 공급되는 전원에 이상이 없는지 여부를 체크해야 한다.)
 
![2](https://user-images.githubusercontent.com/58063370/73435149-73c49680-438b-11ea-8469-9e7eca8ac464.png)

그림 9. HerkuleX Node실행 시 터미널 출력화면.

	HerkuleX Node에서는 총 14가지 타입의 입력으로 HerkuleX의 수동 조작이 가능하다.

	입력인자는 총 4가지이며, 각각의 인자는 ‘,’(comma)로 구분한다.  첫 번째 인자로 명령어의 타입을 결정하며, 두 번째 인자로 HerkuleX의 ID값, 세 번째는 Register의 주소번지, 네 번째는 데이터 값이다. 각각의 명령 타입마다 이 4가지를 모두 사용하는 경우도 있지만, 그렇지 않은 경우도 있다. 명령어를 수행하는데 필요 없는 입력인자의 경우 ‘0’을 입력해 주면 된다. 이 14가지 타입의 입력에 대한 사용방법 및 설명은 다음과 같다. (모든 입력인자를 입력한 후에는 ‘Enter’키를 입력한다.)

①	: HerkuleX의 모터 토크를 On/Off하거나 브레이크 On시킬 수 있는 명령어이다.

예_1) 1,1,0,1을 입력하는 경우 HerkuleX ID 1번에 모터토크를 On시킨다.

예_2) 1,1,0,0을 입력하는 경우 HerkuleX ID 1번에 모터토크를 Off시킨다.

예_3) 1,1,0,2을 입력하는 경우 HerkuleX ID 1번에 브레이크를 On시킨다.
	
 
②	: HerkuleX에 속도모드 지령을 전달하는 명령어이다.

예) 2,1,0,100을 입력하는 경우 HerkuleX ID1번을 목표속도 100(100*11.2msec)으로 동작한다.
	
③	: HerkuleX에 위치모드 지령을 전달하는 명령어이다.

예) 3,1,100,512을 입력하는 경우 HerkuleX ID1번을 100(100*11.2ms)시간 안에 목표위치 512로 동작한다.

④	: HerkuleX의 RAM Register 주소를 선택하여 해당 데이터를 확인하는 명령어이다.

예) 4,1,60,0을 입력하는 경우 HerkuleX ID1번의 절대위치(Absolute Position) 데이터를 확인한다.

. 해당 명령이 수행되면 선택된 주소의 RAM_MAP topic의 값만 업데이트 된다.

⑤	: HerkuleX의 EEP Register 주소를 선택하여 해당 데이터를 확인하는 명령어이다.

예) 5,1,50,0을 입력하는 경우 HerkuleX ID1번의 위치에러 허용치(Inposition Margin) 데이터를 확인한다.

. 해당 명령이 수행되면 선택된 주소의 EEP_MAP topic의 값만 업데이트 된다.

⑥	: HerkuleX의 RAM Register 주소를 선택하여 사용자가 원하는 데이터를 입력하는 명령어이다.

예) 6,1,53,1을 입력하는 경우 HerkuleX ID1번에LED Control값에 1을 입력하여 녹색 LED를 On.

⑦	: HerkuleX의 EEP Register 주소를 선택하여 사용자가 원하는 데이터를 입력하는 명령어이다.

예) 7,1,50,3을 입력하는 경우 HerkuleX ID1번에 위치에러 허용치(Inposition Margin) 데이터를 3으로 적용한다.

⑧	: HerkuleX의 모든 RAM Register 데이터를 읽어서 확인하는 명령어이다.

예) 8,1,0,0을 입력하는 경우 HerkuleX ID1번에 모든 RAM Register데이터를 읽어들인다.

. 해당 명령이 수행되면 RAM_MAP topic의 모든 값이 업데이트 된다.

⑨	: HerkuleX의 모든 EEP Register 데이터를 읽어서 확인하는 명령어이다.

예) 9,1,0,0을 입력하는 경우 HerkuleX ID1번에 모든 EEP Register데이터를 읽어들인다.

. 해당 명령이 수행되면 EEP_MAP topic의 모든 값이 업데이트 된다.

⑩	: HerkuleX의 재 부팅을 수행하는 명령어이다. (HW전원 Off/On와 동일한 기능)

예) 10,1,0,0을 입력하는 경우 HerkuleX ID1번에 전원 Off -> On이 수행된다.

[주의] RAM Register에 기록된 값은 모두 저장되지 않음. (휘발성 메모리 영역)

⑪	: HerkuleX의 RAM과 EEP Register의 값을 공장초기화 상태로 되돌리는 명령어

예) 11,1,0,0을 입력하는 경우 HerkuleX ID1번에 RAM&EEP Register데이터의 초기화 수행.

단, ID와 Board rate(bps)의 값은 변경되지 않고 유지됨.

⑫	: PC에 연결된 HerkuleX의 ID를 Scan하는 명령어로서 1초 간격으로 ID 1번에서 253번까지의 ID를 Scan한다.

예) 12,0,0,0을 입력하는 경우 HerkuleX의 ID는 모두 Scan한다.

⑬	: PC에서 HerkuleX로 송/수신하는 명령어들의 Packet정보를 터미널 창에 출력하는 기능을 활성화 한다. 

예) 13,0,0,0을 입력하면 해당 기능이 활성화 된다.
 
![3](https://user-images.githubusercontent.com/58063370/73435266-a8385280-438b-11ea-9c4c-27330443d14f.png)
 
그림 10. Packet 송/수신 예시화면

. 해당 기능활성화 후에 4,1,0,0(HerkuleX ID1번에 RAM Register 0번 읽기)을 입력하는 경우, 위의 그림과 같이 송신된 Packet의 정보와 수신된 Packet의 정보가 터미널 창에 출력된다. 출력되는 Packet의 값은 16진수(Hex)의 값으로 출력된다.

⑭	: PC에서 HerkuleX로 전달하는 명령어들의 Packet정보를 터미널 창에 출력하는 기능을 비활성화 한다.

예) 14,0,0,0을 입력하면 해당 기능이 비활성화 된다.








