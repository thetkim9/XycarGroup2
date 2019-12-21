import numpy as np
import rospy
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from YOLO.msg import yoloMessage

class remoteYolo:
    def __init__(self, From, To):
        rospy.init_node('remoteYolo')
        self.bridge = CvBridge()
        self.pubYolo = rospy.Publisher(To, yoloMessage, queue_size=1)
        rospy.Subscriber(From, Image, self.doYolo)

    def doYolo(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        yolo_path = '/home/user/darknet/python/yolo-coco'
        yolo_confidence = 0.5
        yolo_threshold = 1.0

        # load the COCO class labels our YOLO model was trained on
        labelsPath = os.path.sep.join([yolo_path, "coco.names"])
        LABELS = open(labelsPath).read().strip().split("\n")

        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
                                   dtype="uint8")

        # derive the paths to the YOLO weights and model configuration
        weightsPath = os.path.sep.join([yolo_path, "yolov3.weights"])
        configPath = os.path.sep.join([yolo_path, "yolov3.cfg"])

        # load our YOLO object detector trained on COCO dataset (80 classes)
        # print("[INFO] loading YOLO from disk...")
        # print(configPath, weightsPath)
        # print(cv2.__version__)
        net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

        # load our input image and grab its spatial dimensions
        # image = cv2.array(eval(image_data), dtype='uint8')
        image = frame
        (H, W) = image.shape[:2]

        # determine only the *output* layer names that we need from YOLO
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        # construct a blob from the input image and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes and
        # associated probabilities
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        net.setInput(blob)
        layerOutputs = net.forward(ln)

        # show timing information on YOLO
        # print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > yolo_confidence:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, yolo_confidence,
                                yolo_threshold)

        ret = []

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                ret.append(LABELS[classIDs[i]])

        yoloInfo = ret
        pub_info = yoloMessage(data=yoloInfo)
        self.pubYolo.publish(pub_info)



if __name__=="__main__":
    From = '/usb_cam/image_raw'
    To = "/yolo"
    yolo = remoteYolo(From, To)


