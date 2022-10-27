#!/usr/bin/env python

import rospy, signal
import numpy as np
from functools import partial
from matplotlib import cm

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import quaternion_from_euler

from timer import Timer
from visualizerclass import VisualizerClass
from mahony import Mahony
from extended_kalman import EKF
from utils import *

Hz = 50
apriltag_x = 0.0; apriltag_y = 0.0; apriltag_yaw = 0.0

acc_x = 0.0; acc_y = 0.0; acc_z = 0.0
gyro_x = 0.0; gyro_y = 0.0; gyro_z = 0.0

flag = 1

D2R = np.pi / 180
R2D = 180 / np.pi


def callback(data):
    global apriltag_x, apriltag_y, apriltag_yaw 
    array = data.data.split()
    apriltag_x = float(array[0])
    apriltag_y = float(array[1])
    apriltag_yaw = float(array[2])

def get_position():
    return apriltag_x, apriltag_y, apriltag_yaw

def callback2(data):
    global acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
    array = data.data.split()
    acc_x = -float(array[1])
    acc_y = float(array[0])
    acc_z = float(array[2])
    gyro_x = float(array[3])
    gyro_y = float(array[4])
    gyro_z = float(array[5])

def get_acc():
    return acc_x, acc_y, acc_z

def get_gyro():
    return gyro_x, gyro_y, gyro_z

def callback3(data):
    global flag
    array = data.data.split()
    flag = int(array[0])

def get_flag():
    return flag

def signal_handler(timer, signal, frame):
    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary
  

if __name__ == '__main__':
        
    # Initialize node
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("apriltag_position", String, callback)
    rospy.Subscriber("imu_sensor", String, callback2)
    rospy.Subscriber("flag", String, callback3)

    tmr_plot = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    
    rs_pos_data = np.empty(shape=(0,2))
    acc_data = []
    gyro_data = []

    traj = np.load("trajectory.npy")
    x_traj = traj[:,0]
    y_traj = traj[:,1]

    flag_array = []

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=Hz)
    
    # Start the loop 
    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (HZ)
            # V.reset_markers()            
            # V.reset_texts()

            check_flag = get_flag()
            flag_array.append(check_flag)
            x = [0]; y = [0]
            
            if flag_array[-1]:
                rs_x, rs_y, april_yaw = get_position()
                rs_pos_data = np.append(rs_pos_data, np.array([[rs_x, rs_y]]), axis=0)
                if tmr_plot.tick > 1:
                    x.append(-rs_y + rs_pos_data[1, 1])
                    y.append(rs_x - rs_pos_data[1, 0])
                    print(x[-1],y[-1])
        
            else:
                rs_pos_data = np.array([[0, 0]])
                x = [0]; y = [0]; april_yaw = 0
                acc_data = []
                gyro_data = []
                V.reset_markers()            
                V.reset_texts()

            ax, ay, az = get_acc()
            acc_data.append([ax, ay, az])
            text = "Acc_x: %f Acc_y: %f Acc_z: %f"%(ax, ay, az)

            gx, gy, gz = get_gyro()
            gyro_data.append([gx, gy, gz])

            orientation_mahony = Mahony(gyr=gyro_data[-40:], acc=acc_data[-40:])
            q_mahony = orientation_mahony.Q[-1,:]

            roll1, pitch1, yaw1 = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
            yaw1 = april_yaw

            # V.append_marker(Quaternion(*quaternion_from_euler(-roll1,-pitch1,yaw1)),Vector3(0.2,0.06,0.06),x=x[-1],y=y[-1],z=0,frame_id='map',
            #     color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)

            # V.append_linestrip(x_array=x_traj,y_array=y_traj,z=0.0,scale=Vector3(0.01,0,0),
            #     frame_id='map',color=ColorRGBA(1.0,1.0,1.0,1.0),marker_type=Marker.LINE_STRIP)

            V.set_marker_line(Q=Quaternion(*quaternion_from_euler(-roll1,-pitch1,yaw1)),marker_x=x[-1],marker_y=y[-1],marker_z=0,
            line_x=x_traj,line_y=y_traj,line_z=0,
            marker_scale=Vector3(0.2,0.06,0.06),line_scale=Vector3(0.01,0,0),frame_id='map',
            marker_color=ColorRGBA(1.0,0.0,0.0,0.5),line_color=ColorRGBA(1.0,1.0,1.0,1.0),
            marker_type=Marker.ARROW,line_type=Marker.LINE_STRIP)

            V.publish_markers()
            V.publish_texts()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    V.delete_texts()

##rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10

