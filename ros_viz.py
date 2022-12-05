#!/usr/bin/env python

import os
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, String

from classes.timer import Timer
from classes.visualizerclass import VisualizerClass
from classes.class_snapbot import quaternion_to_vector
from filter.mahony import Mahony
from filter.extended_kalman import EKF

Hz = 50
D2R = np.pi/180
R2D = 180/np.pi

class ApriltagData():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_tag = "apriltag_position"
        self.isReady_tag = False
        self.sub_tag = rospy.Subscriber(self.topic_sub_tag, String, self.callback)
        while self.isReady_tag is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_tag = True
        array = data.data.split()
        self.x = float(array[0])
        self.y = float(array[1])
        self.yaw = float(array[2])

class IMUData():
    def __init__(self):
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_imu = "imu_sensor"
        self.isReady_imu = False
        self.sub_imu = rospy.Subscriber(self.topic_sub_imu, String, self.callback)
        while self.isReady_imu is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_imu = True
        array = data.data.split()
        self.acc_x = -float(array[1])
        self.acc_y = float(array[0])
        self.acc_z = float(array[2])
        self.gyro_x = float(array[3])
        self.gyro_y = float(array[4])
        self.gyro_z = float(array[5])

class FlagData():
    def __init__(self):
        self.flag = 0
        self.init_subscriber()
    
    def init_subscriber(self):
        self.topic_sub_flag = "flag"
        self.isReady_flag = False
        self.sub_flag = rospy.Subscriber(self.topic_sub_flag, String, self.callback)
        while self.isReady_flag is False: rospy.sleep(1e-3)
    
    def callback(self, data):
        self.isReady_flag = True
        array = data.data.split()
        self.flag = int(array[0])
  
if __name__ == '__main__':
        
    # Initialize node
    rospy.init_node('subscriber', anonymous=True)
    print("Start visualization_engine.")
    CURR_FOLDER = os.getcwd()

    tmr_plot = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    apriltag = ApriltagData()
    imu = IMUData()
    #flag = FlagData()

    rs_pos_data = np.empty(shape=(0,2))
    acc_data = []
    gyro_data = []
    yaw_data = []
    yaw_val = 0.0

    traj = np.load(CURR_FOLDER + '/qpos_dlpg/trajectory.npy')
    x_traj = traj[:,0]
    y_traj = traj[:,1]

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=Hz)

    V.reset_lines()  
    V.append_line(x_array=x_traj,y_array=y_traj,z=0.0,r=0.01,
                frame_id='map',color=ColorRGBA(1.0,1.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
    V.publish_lines()

    # Start the loop 
    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (HZ)

            tick = tmr_plot.tick

            # Reset
            V.reset_markers()
            V.reset_meshes()           
            V.reset_texts()
            x = [0]; y = [0]
            
            #if flag.flag:
            if True:
                rs_pos_data = np.append(rs_pos_data, np.array([[apriltag.x, apriltag.y]]), axis=0)
                if tick > 1:
                    x.append(-apriltag.y + rs_pos_data[1, 1])
                    y.append(apriltag.x - rs_pos_data[1, 0])
                    #print(x[-1],y[-1])
            else:
                tmr_plot.start()
                rs_pos_data = np.array([[0, 0]])
                x = [0]; y = [0]
                acc_data = []
                gyro_data = []
                yaw_data = []
                yaw_val = 0.0
                V.reset_markers()            
                V.reset_texts()

            acc_data.append([imu.acc_x, imu.acc_y, imu.acc_z])
            gyro_data.append([imu.gyro_x, imu.gyro_y, imu.gyro_z])

            orientation_mahony = Mahony(gyr=gyro_data[-40:], acc=acc_data[-40:])
            q_mahony = orientation_mahony.Q[-1,:]

            roll, pitch, yaw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
            yaw_data.append(yaw)
            yaw_val = yaw_val + yaw_data[-1]

            # integrated yaw value (Red Arrow)
            mahony_rpy = "Mahony\nRoll: %f° Pitch: %f° Yaw: %f°"%((np.pi-roll)*R2D,-pitch*R2D,yaw_val*2.5/Hz*R2D)
            V.append_text(x=x[-1],y=y[-1],z=0.3,r=0.1,text=mahony_rpy,
                frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))
            # V.append_marker(x=x[-1],y=y[-1],z=0,frame_id='map',roll=-roll,pitch=-pitch,yaw=yaw_val*2.5/Hz,
            #     scale=Vector3(0.2,0.06,0.06),color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)
            # V.append_line(x_array=x_traj,y_array=y_traj,z=0.0,r=0.01,
            #     frame_id='map',color=ColorRGBA(1.0,1.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
            
            stl_path = 'file://' + CURR_FOLDER + '/ROS_viz_engine/snapbot_low_resol.stl'
            V.append_mesh(x=x[-1],y=y[-1],z=0,scale=1.0,dae_path=stl_path,
                frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                roll=np.pi-roll,pitch=-pitch,yaw=yaw_val*2.5/Hz)

            # apriltag yaw value (Blue Arrow)
            V.append_marker(x=x[-1],y=y[-1],z=0,frame_id='map',roll=-roll,pitch=-pitch,yaw=apriltag.yaw,
                scale=Vector3(0.2,0.06,0.06),color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)          

            # time
            time = "%.2fsec"%(tmr_plot.sec_elps)
            V.append_text(x=0,y=0,z=0.5,r=1.0,text=time,
                frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            V.publish_markers()
            V.publish_meshes()
            V.publish_texts()
            V.publish_lines()
            # tmr_plot.end()

        # V.append_line(x_array=x,y_array=y,z=0.0,r=0.01,
        #     frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
        # V.publish_lines()
        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    V.delete_meshes()
    V.delete_texts()
    V.delete_lines()

##rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10