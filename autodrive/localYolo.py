import rospy
from YOLO.msg import yoloMessage

class localYolo:
    def __init__(self):
        self.reconObjects = []
        rospy.Subscriber("/yolo", yoloMessage, self.readYolo)

    def readYolo(self, data):
        self.reconObjects = data

    def getYolo(self):
        return self.reconObjects
