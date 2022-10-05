#!/usr/bin/env python

import rospy, signal
import numpy as np
from functools import partial
from matplotlib import cm

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA, Int32
from std_msgs.msg import Float64MultiArray

from timer import Timer
from visualizerclass import VisualizerClass

apriltag_x = 0.0
apriltag_y = 0.0

D2R = np.pi / 180
R2D = 180 / np.pi


def callback(data):
    global apriltag_x, apriltag_y
    apriltag_x = data.data[0]
    apriltag_y = data.data[1]

def get_position():
    return apriltag_x, apriltag_y

def signal_handler(timer, signal, frame):
    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary
  

if __name__ == '__main__':
        
    # Initialize node
    rospy.init_node('apriltag_subscriber', anonymous=True)
    rospy.Subscriber("apriltag_position", Float64MultiArray, callback)

    tmr_plot = Timer(_name='Plot',_HZ=20,_MAX_SEC=np.inf,_VERBOSE=True)
    
    # Set terminator
    signal.signal(signal.SIGINT, partial(signal_handler,tmr_plot))

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=20)
    
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
            x = 5 * x
            y = 5 * y
            print(x,y)

            V.append_marker(x=x,y=y,z=0,r=0.5,frame_id='map',
                color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)

            # Append text
            text = "%f %f"%(x,y)
            V.append_text(x=x,y=y,z=1.3,r=1.0,text=text,
               frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            # Publish
            V.publish_markers()
            V.publish_texts()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    #V.delete_texts()

##rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10