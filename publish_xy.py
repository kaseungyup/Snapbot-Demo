import cv2
import os
import numpy as np
import time
import math
import apriltag
import pyrealsense2 as rs
from realworld_func.class_motionhelper import tracker, timer
from utils_real import *
from utils_track import tps_trans, get_tps_mat

Hz = 50
tps_coef = get_tps_mat()

if __name__ == '__main__':
    publish_xy(tps_coef, Hz, 300, LOG_INFO = True, VERBOSE = True)