## Introduction to Creative Engineering Design AD Project

--------

### 프로젝트 설명

* <b>차선인식 자율주행</b>: 기존 창업연계공학설계입문과 유레카프로젝트에서 진행되었던 차선인식 자율주행을 기반으로 자율주행스튜디오의 트랙완주를 기본 Base로 한다. 정확도를 높여 중앙선을 기준으로 매끄럽게 직선구간, 4번의 코너구간과, S자 구간을 이탈하지 않고 빠른속도로 완주한다. 

* <b>정지 표지판 인식시 정지후 출발</b>: YOLO 알고리즘을 사용하여 카메라를 통해 정지 표지판을 인식하면, 일정거리를 남겨두고 정지하였다가 출발한다. 

* <b>장애물 탐지시 정지후 출발</b>: Xycar의 전방 초음파 센서를 통해 장애물을 탐지할 경우 자율 구동체를 즉시 정지시키고, 장애물이 사라지면 다시 주행한다. 

* <b>오르막길 인식시 가감속</b>: IMU센서를 활용해 자율구동체의 언덕 주행상황을 인지해 오르막길에서는 증속을, 내리막길에서는 감속하는등 적절한 속도 제어를 하고자 한다. 

### 패키지 설명

* `catkin_ws`: YOLO 인식을 xycar가 아닌 thinkpad에서 원격으로 하여 xycar에 그 정보를 보내주기 위해 생성된 catkin workspace입니다.

* `testYolo`는 solution.py를 실행시켜 yolo.py가 잘 동작하는지를 확인하는데 사용한 디렉토리입니다(GPU 기반으로도 잘 인식이 되는지).

* `autodrive`: Xycar의 디렉토리 입니다.

* /catkin_ws/src/YOLO/src/yolo-colo와 testYolo/yolo-colo 내부에는 yolov3.weights라는 darknet로부터 가져온 pretrained neural network model의 weight값이 들어있지만, 파일의 사이즈가 100MB를 초과하여 github에는 올릴 수 없게 되었습니다.