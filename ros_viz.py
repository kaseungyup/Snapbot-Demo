#!/usr/bin/env python

import rospy, signal
import numpy as np
from functools import partial
from matplotlib import cm

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Vector3

from timer import Timer
from visualizerclass import VisualizerClass

apriltag_x = 0.0
apriltag_y = 0.0

acc_x = 0.0
acc_y = 0.0
acc_z = 0.0

D2R = np.pi / 180
R2D = 180 / np.pi


def callback(data):
    global apriltag_x, apriltag_y
    array = data.data.split()
    apriltag_x = float(array[0])
    apriltag_y = float(array[1])

def get_position():
    return apriltag_x, apriltag_y

def callback2(data):
    global acc_x, acc_y, acc_z
    array = data.data.split()
    acc_x = float(array[0])
    acc_y = float(array[1])
    acc_z = float(array[2])

def get_acc():
    return acc_x, acc_y, acc_z

def signal_handler(timer, signal, frame):
    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary
  

if __name__ == '__main__':
        
    # Initialize node
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("apriltag_position", String, callback)
    rospy.Subscriber("imu_sensor", String, callback2)

    tmr_plot = Timer(_name='Plot',_HZ=50,_MAX_SEC=np.inf,_VERBOSE=True)
    
    # Set terminator
    signal.signal(signal.SIGINT, partial(signal_handler,tmr_plot))

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=50)
    
    # Start the loop 

    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (20HZ)

            tick = tmr_plot.tick

            # Reset 
            V.reset_markers()            
            V.reset_texts()
            
            # Append marker
            x,y = get_position()
            x = 10 * x
            y = 10 * y

            V.append_marker(x=x,y=y,z=0,r=0.5,frame_id='map',
                color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)

            # Append text
            ax, ay, az = get_acc()
            text = "Acc_x: %f Acc_y: %f Acc_z: %f"%(ax,ay, az)
            V.append_text(x=x,y=y,z=1.3,r=1.0,text=text,scale=Vector3(0,0,0.3),
               frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            # Publish
            V.publish_markers()
            V.publish_texts()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    V.delete_texts()

##rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10