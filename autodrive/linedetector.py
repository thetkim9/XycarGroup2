import cv2
import numpy as np
try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except:
    pass

class LineDetector:

    def __init__(self, topic, ros_node=True):
        self.ros_node = ros_node
        self.image_width = 640
        self.scan_width, self.scan_height = 400, 40
        self.area_width, self.area_height = 2,2
        area = self.area_width * self.area_height
        self.pxl_cnt_threshold = area * 0.1
        self.linescan_offset = 15
        self.roi_vertical_pos = 300
        self.left, self.right = -1, -1
        self.stop = 0

        self.before = [0, 0, 0]

        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        if self.ros_node:
            self.bridge = CvBridge()
            rospy.Subscriber(topic, Image, self.conv_image)
            self.recorder = cv2.VideoWriter(
                '/home/nvidia/xycar/src/auto_drive/record.avi',
                cv2.VideoWriter_fourcc(*'MJPG'),
                30,
                (640, 480)
            )

    def __del__(self):
        if self.ros_node:
            self.recorder.release()
        cv2.destroyAllWindows()

    def conv_image(self, data):
        if self.ros_node:
            self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.recorder.write(self.cam_img)
        else:
            self.cam_img = data

        stop_roi = self.cam_img[100:220, 50:230]
        hsv2 = cv2.cvtColor(stop_roi, cv2.COLOR_BGR2HSV)
        slbound = np.array([150, 0, 0], dtype=np.uint8)
        subound = np.array([255, 255, 255], dtype=np.uint8)
        self.stop_mask = cv2.inRange(hsv2, slbound, subound)

        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)

        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)
        self.linesP = cv2.HoughLinesP(self.edge, 1, np.pi / 180, 50, None, 50, 10)

        self.cdst = cv2.cvtColor(self.edge, cv2.COLOR_GRAY2BGR)
        self.cdstP = np.copy(self.cdst)

        if self.linesP is not None:
            for i in range(0, len(self.linesP)):
                l = self.linesP[i][0]
                cv2.line(self.cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

        self.finalMask = cv2.inRange(self.cdstP, np.array([0, 0, 255], dtype=np.uint8), np.array(
            [0, 0, 255], dtype=np.uint8))

    def detect_lines(self):
        self.left, self.right = -1, -1
        self.stop = 0
        for l in range(200, self.area_width, -1):
            area = self.finalMask[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pxl_cnt_threshold:
                self.left = l
                break
        for r in range(440, self.image_width - self.area_width):
            area = self.finalMask[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pxl_cnt_threshold:
                self.right = r
                break

        for s in range(10):
            if cv2.countNonZero(self.stop_mask) > 1000:
                self.stop = 1
                break

        return self.left, self.right, self.stop

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().

        cv2.waitKey(1)

        angle = 0

        mid = (left + right) // 2
        print(left, right, mid)
        if left == -1 and right == -1:
            left, right, mid = self.before[0], self.before[1], self.before[2]
            print(self.before)

        # if right != -1 and (self.before[2]-mid) > 5:
        #     mid = (mid + self.before[2]) * 1//2+5
        #     print((mid + self.before[2]) * 1//2+5)
        print(cv2.countNonZero(self.stop_mask), self.stop)

        if self.stop == 1:
            print("stopppppppppp")

        if left == -1:
            if mid - 300 < -50:
                angle = -50
            else:
                angle = mid - 300
        elif right == -1:
            if mid - 15 > 50:
                angle = 50
            else:
                angle = mid - 15
        elif mid > 325:
            if mid - 325 > 50:
                angle = 50
            else:
                angle = mid - 325
        elif mid < 295:
            if mid - 295 < -50:
                angle = -50
            else:
                angle = mid - 295
        else:
            angle = 0

        print(angle)

        self.before[0], self.before[1], self.before[2] = left, right, mid

        cv2.imshow("origin", self.cam_img)
        cv2.imshow("view", self.mask)
        cv2.imshow('egde', self.edge)
        cv2.imshow('1', self.finalMask)
        cv2.imshow('2', self.stop_mask)
