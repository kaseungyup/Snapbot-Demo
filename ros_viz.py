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

Hz = 100
apriltag_x = 0.0; apriltag_y = 0.0; apriltag_yaw = 0.0

acc_x = 0.0; acc_y = 0.0; acc_z = 0.0
gyro_x = 0.0; gyro_y = 0.0; gyro_z = 0.0

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
    # acc_x = float(array[1])
    # acc_y = float(array[2])
    # acc_z = -float(array[0])
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

def signal_handler(timer, signal, frame):
    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary
  

if __name__ == '__main__':
        
    # Initialize node
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("apriltag_position", String, callback)
    rospy.Subscriber("imu_sensor", String, callback2)

    tmr_plot = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    
    acc_data = []
    Earth_frame_acc_data = []
    gyro_data = []
    quat_data = []
    vel_data = []
    vel_data.append(np.array([0, 0, 0]))
    pos_data = []
    pos_data.append(np.array([0, 0, 0]))

    # Set terminator
    # signal.signal(signal.SIGINT, partial(signal_handler,tmr_plot))

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=Hz)
    
    # Start the loop 

    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (20HZ)

            tick = tmr_plot.tick

            V.reset_markers()            
            V.reset_texts()
            
            x,y, april_yaw = get_position()
            x = 10 * x
            y = 10 * y
            #print(x,y)

            ax, ay, az = get_acc()
            acc_data.append([ax, ay, az])
            text = "Acc_x: %f Acc_y: %f Acc_z: %f"%(ax, ay, az)

            gx, gy, gz = get_gyro()
            gyro_data.append([gx, gy, gz])

            orientation_mahony = Mahony(gyr=gyro_data[-40:], acc=acc_data[-40:])
            q_mahony = orientation_mahony.Q[-1,:]

            Earth_frame_acc = q_rot(q_conj(q_mahony), np.array([ax, ay, az])) - np.array([0,0,9.81])
            Earth_frame_acc_data.append(np.array(Earth_frame_acc))
            vel, pos = update_position(Earth_frame_acc[-40:], vel_data[-40:], pos_data[-40:], Hz)
            #print("vel: ", vel)
            #print("pos: ", pos)

            roll1, pitch1, yaw1 = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
            #yaw = april_yaw
            #print(roll,pitch,yaw)

            V.append_marker(Quaternion(*quaternion_from_euler(-roll1,-pitch1,yaw1)),Vector3(1,0.3,0.3),x=x,y=y,z=0,frame_id='map',
                color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)
            # text1 = "Mahony\nRoll: %f Pitch: %f Yaw: %f"%(-roll1*R2D,-pitch1*R2D,yaw1*R2D)
            # V.append_text(x=2,y=0,z=1.3,r=1.0,text=text1,scale=Vector3(0,0,0.3),
            #     frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            # orientation_kalman = EKF(gyr=gyro_data[-40:], acc=acc_data[-40:], mag=None)
            # q_kalman = orientation_kalman.Q[-1,:]
            # roll2, pitch2, yaw2 = quaternion_to_vector(q_kalman[0],q_kalman[1],q_kalman[2],q_kalman[3])
            # V.append_marker(Quaternion(*quaternion_from_euler(-roll2,-pitch2,yaw2)),Vector3(1,0.3,0.3),x=-2,y=0,z=0,frame_id='map',
            #     color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)
            # text2 = "Kalman\nRoll: %f Pitch: %f Yaw: %f"%(-roll2*R2D,-pitch2*R2D,yaw2*R2D)
            # V.append_text(x=-2,y=0,z=1.3,r=1.0,text=text2,scale=Vector3(0,0,0.3),
            #     frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            V.append_text(x=x,y=y,z=1.3,r=1.0,text=text,scale=Vector3(0,0,0.3),
                frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            V.publish_markers()
            V.publish_texts()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    V.delete_texts()

##rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10

