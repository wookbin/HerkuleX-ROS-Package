[HerkuleX 6개(그리퍼 제외)와 HerkuleX전용 브라켓을 이용한 6DOF Arm 예제]

-	해당 예제는 ROS에서 제공하는 Moveit을 활용한 예제로서 HerkuleX_6DOF_Arm_node.cpp에 6개의 HerkuleX를 동작하기 위한 코드가 구현되어 있으며, Moveit Setup Assistant를 이용하여 생성한 moveit_HerkuleX_Arm패키지를 이용하여 동작을 할 수 있습니다

※	Moveit Setup Assistant에 대해서는 아래 링크를 참고해 주세요.

 http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

-	6DOF Arm의 동작을 위해서는 아래그림과 같이 2개의 launch파일을 실행해 주어야 합니다.

![11](https://user-images.githubusercontent.com/58063370/76718844-5a30b000-677b-11ea-8676-e57921d19748.png)

-	위의 그림과 같이 2개의 launch를 실행하면, 아래그림과 같이 RVIZ화면에 6DOF Arm의 화면이 출력됩니다.

![12](https://user-images.githubusercontent.com/58063370/76718850-5d2ba080-677b-11ea-8bea-14545ce780dc.png)

-	Arm의 끝 단에 위치한 작용점 부분을 원하는 위치로 이동 시킨 후 Planning 탭에 있는 Plan and Execute버튼을 클릭하면, Arm이 움직이는 경로에 대한 시뮬레이션과 함께, 실제 Arm이 동작합니다.

-	Topic의 구성은 아래그림과 같습니다.

![13](https://user-images.githubusercontent.com/58063370/76718853-5f8dfa80-677b-11ea-95c5-7a7191edb169.png)
