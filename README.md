catkin_ws는 YOLO 인식을 xycar가 아닌 thinkpad에서 원격으로 하여 xycar에 그 정보를 보내주기 위해 생성된 catkin workspace입니다.
testYolo는 solution.py를 실행시켜 yolo.py가 잘 동작하는지를 확인하는데 사용한 디렉토리입니다(GPU 기반으로도 잘 인식이 되는지).
autodrive.launch는 본래 /home/nvidia/xycar/src/auto_drive/launch 안에 있어야 하는 파일이지만, 전체 auto_drive를 xycar에서 가지고 오지 못해서 여기 놔두었습니다.
/catkin_ws/src/YOLO/src/yolo-colo와 testYolo/yolo-colo 내부에는 yolov3.weights라는 darknet로부터 가져온 pretrained neural network model의 weight값이 들어있지만, 파일의 사이즈가 100MB를 초과하여 github에는 올릴 수 없게 되었습니다.
