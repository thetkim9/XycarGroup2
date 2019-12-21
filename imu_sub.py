#!/usr/bin/env python

import rospy, time
from diagnostic_msgs.msg import DiagnosticArray

imu_data = None

def callback(data):
    global imu_data
    imu_data = data.status[0].values

rospy.init_node('IMU_subscriber', anonymous=True)
rospy.Subscriber('/diagnostics', DiagnosticArray, callback, queue_size=1)

time.sleep(15)

while not rospy.is_shutdown():
    roll = imu_data[0].value
    pitch = imu_data[1].value
    yaw = imu_data[2].value
    print("R {} P {} Y {}".format(roll, pitch, yaw))
    time.sleep(1)