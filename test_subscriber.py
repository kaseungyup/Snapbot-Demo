#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray

# acc_x = 0.0
# acc_y = 0.0
# acc_z = 0.0

# x = 0.0
# y = 0.0

# def callback(data):
    # global acc_x, acc_y, acc_z
    # array = data.data.split()
    # acc_x = float(array[0])
    # acc_y = float(array[1])
    # acc_z = float(array[2])
    # print(acc_x, acc_y, acc_z)
    # print(data)
   
# def get_acc():
#     return acc_x, acc_y, acc_z

# def callback2(data):
#     global x,y
#     x = data.data[0]
#     y = data.data[1]
#     print(x, y)

# if __name__ == '__main__':
#     rospy.init_node('subscriber', anonymous=True)
#     rospy.Subscriber("imu_sensor", String, callback)
#     # rospy.Subscriber("apriltag_position", Float64MultiArray, callback2)
#     rospy.spin()

from timer import Timer
from datetime import datetime

a = 0
b = 0

def callback(data):
    global a, b
    array = data.data.split()
    a = float(array[0])
    b = float(array[1])

def get_data():
    return a, b

if __name__ == "__main__":
    rospy.init_node("test_subscriber", anonymous=True)
    rospy.Subscriber("test", String, callback)
    
    tmr_plot = Timer(_name='Plot',_HZ=1,_MAX_SEC=np.inf,_VERBOSE=True)
    tmr_plot.start()

    temparray = np.empty(shape=(0,2))
    
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (20HZ)
            rx, ry = get_data()
            #print("rx: %s, ry: %s"%(rx,ry))
            temparray = np.append(temparray, np.array([[rx, ry]]), axis=0)
            if tmr_plot.tick > 1: 
                x = rx - temparray[1,0]
                y = ry - temparray[1,1]
                print(x,y)
                