import rospy
from diagnostic_msgs.msg import DiagnosticArray

class ImuRead:
    def __init__(self, topic):
        self.roll = -1
        self.pitch = -1
        self.yaw = -1
        rospy.Subscriber(topic, DiagnosticArray, self.read_data)

    def read_data(self, data):
        status = data.status[0].values
        self.roll = status[0].value
        self.pitch = status[1].value
        self.yaw = status[2].value

    def get_data(self):
        return float(self.roll), float(self.pitch), float(self.yaw)