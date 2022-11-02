import cv2
import os
import numpy as np
import time
import math
import apriltag
import pyrealsense2 as rs
from realworld_func.class_motionhelper import timer
from realworld_func.class_xm430 import xm430
from utils.utils_track import tps_trans
from publisher import apriltag_publisher, flag_publisher
# import matplotlib.patches as patches 
# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('agg')

def prepare_snapbot(leg_idx):
    usb = 0
    for i in range(10):
        if os.path.exists("/dev/ttyUSB{}".format(i)):
            usb = i
    os.system("sudo chmod a+rw /dev/ttyUSB{}".format(usb))
    snapbot = xm430('SNAPBOT', USB_NUM=usb)
    snapbot.connect()
    snapbot.IDX_LIST = leg_idx
    snapbot.set_delaytime([30])
    snapbot.set_pidgains(20, 0, 0)
    snapbot.set_maxtorque([2000])
    snapbot.set_goalspeed([1500])
    snapbot.set_torque([1])
    return snapbot

def get_traj(qpos):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos.T[7] = -qpos.T[7]
    qpos = 2048/180*qpos+2048

    # 41 degree 
    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[:, 0] = np.clip(qpos[:, 0], minus_joint_limit, plus_joint_limit)
    qpos[:, 2] = np.clip(qpos[:, 2], minus_joint_limit, plus_joint_limit)
    qpos[:, 4] = np.clip(qpos[:, 4], minus_joint_limit, plus_joint_limit)
    qpos[:, 6] = np.clip(qpos[:, 6], minus_joint_limit, plus_joint_limit)
    return qpos

def get_traj_3(qpos):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos = 2048/180*qpos+2048

    # 41 degree 
    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[:, 0] = np.clip(qpos[:, 0], minus_joint_limit, plus_joint_limit)
    qpos[:, 2] = np.clip(qpos[:, 2], minus_joint_limit, plus_joint_limit)
    qpos[:, 4] = np.clip(qpos[:, 4], minus_joint_limit, plus_joint_limit)
    return qpos

def get_shorttraj(qpos):
    poses = get_traj(qpos)
    short_traj = poses[0:300,:]
    return short_traj

def find_snapbotcamidx():
    all_camera_idx_available = []
    for camera_idx in range(10):
        cap = cv2.VideoCapture(camera_idx)
        if cap.isOpened():
            all_camera_idx_available.append(camera_idx)
            cap.release()
        else : 
            pass
    return all_camera_idx_available[-1]

def Rotate(src, degrees):
    if degrees == 90:
        dst = cv2.transpose(src)
        dst = cv2.flip(dst, 1)
    elif degrees == 180:
        dst = cv2.flip(src, -1)
    elif degrees == 270:
        dst = cv2.transpose(src)
        dst = cv2.flip(dst, 0)
    else:
        dst = src
    return dst

def run_snapbot(qpos, snapbot, Hz, max_sec):
    traj = get_traj(qpos)
    t = timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    idx, threshold= 0, 0
    flag = False
    flag_publisher(1)
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist()))
            # time.sleep(0.001)
            if idx == traj.shape[0]:
                t.finish()
    print("FINISHED")
    flag_publisher(0)

def run_snapbot_3(traj, snapbot, Hz, max_sec):
    t = timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    idx, threshold= 0, 0
    flag = False
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist()))
            time.sleep(0.001)
            if idx == traj.shape[0]:
                t.finish()
    print("FINISHED")

def run_snapbot_get_traj_real_score(qpos, tps_coef, snapbot, Hz, max_sec, VERBOSE=False):
    traj = get_traj(qpos)
    real_xy_y_traj = np.empty(shape=(0,2))

    x = np.linspace(170,470,4)
    y = np.linspace(90,390,4)
    X, Y = np.meshgrid(x,y)
    ctrl_xy = np.stack([X,Y],axis=2).reshape(-1,2)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    DETECT = False

    while not DETECT:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            initPos_x = int(r.center[0])
            initPos_y = int(r.center[1])
            inittany = abs(((ptC[1]+ptD[1])/2) - initPos_y)
            inittanx = abs(((ptC[0]+ptD[0])/2) - initPos_x)
            initrad = math.atan2(inittanx,inittany)
            initdeg = int(initrad * 180 / math.pi)
        init_pos = np.array([[initPos_x,initPos_y]])
        real_init_pos = tps_trans(init_pos, ctrl_xy, tps_coef)
        if init_pos.shape[0] == 1:
            DETECT = True

    time.sleep(1)
    idx, flag, threshold = 0, False, 0
    t = timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist())); time.sleep(0.001)
            if idx == traj.shape[0]:
                t.finish()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)

        if len(results) == 0:
            real_xy_y_traj = np.append(real_xy_y_traj, np.array([[cali_y, -cali_x]]), axis=0)

        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            if VERBOSE : 
                cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            center_pos = np.array([[cX, cY]])
            real_center_pos = tps_trans(center_pos, ctrl_xy, tps_coef)
            if VERBOSE : 
                cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
            tany = abs(((ptC[1]+ptD[1])/2) - cY)
            tanx = abs(((ptC[0]+ptD[0])/2) - cX)
            rad = math.atan2(tanx,tany)
            deg = int(rad * 180 / math.pi)
            cali_x = real_center_pos[0, 0]-real_init_pos[0, 0]
            cali_y = real_center_pos[0, 1]-real_init_pos[0, 1]
            cali_deg = deg - initdeg
            real_xy_y_traj = np.append(real_xy_y_traj, np.array([[cali_y, -cali_x]]), axis=0)
            # draw the tag family on the image
            if VERBOSE: 
                cv2.putText(color_image, "({},{}),{}".format(cX, cY, deg), (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if VERBOSE: 
            cv2.imshow("Frame", color_image)
            if cv2.waitKey(20) == 27:
                break   
    if VERBOSE:
        cv2.destroyAllWindows()

    # list_for_plot = []
    # fig, ax = plt.subplots()
    # ax.set_xlim(0, 2)
    # ax.set_ylim(-0.5, 0.5)
    # for i in real_xy_y_traj:
    #     list_for_plot.append([i[0], i[1]])
    #     ax.add_patch(patches.Rectangle((i[0], i[1]), 0.03, 0.03, facecolor='red', fill=True))
    # plt.show()

    real_score = real_xy_y_traj[-1, 0] - real_xy_y_traj[0, 0]
    print ("FINISHED.")

    return real_xy_y_traj, real_score

def run_snapbot_tps(qpos, tps_coef, snapbot, Hz, max_sec, VERBOSE=False, ROS_publish=True):
    traj = get_traj(qpos)
    real_xy_y_traj = np.empty(shape=(0,2))

    x = np.linspace(170,470,4)
    y = np.linspace(90,390,4)
    X, Y = np.meshgrid(x,y)
    ctrl_xy = np.stack([X,Y],axis=2).reshape(-1,2)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    DETECT = False

    time.sleep(1)
    idx, flag, threshold = 0, False, 0
    t = timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist())); time.sleep(0.001)
            if idx == traj.shape[0]:
                t.finish()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)

        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            if VERBOSE : 
                cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            center_pos = np.array([[cX, cY]])
            real_center_pos = tps_trans(center_pos, ctrl_xy, tps_coef)
            if VERBOSE : 
                cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
            tany = abs(((ptC[1]+ptD[1])/2) - cY)
            tanx = abs(((ptC[0]+ptD[0])/2) - cX)
            rad = math.atan2(tanx,tany)
            deg = int(rad * 180 / math.pi)
            if ROS_publish: apriltag_publisher(real_center_pos[0, 1], -real_center_pos[0, 0], Hz)
            
            if VERBOSE: 
                cv2.putText(color_image, "({},{}),{}".format(cX, cY, deg), (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if VERBOSE: 
            cv2.imshow("Frame", color_image)
            if cv2.waitKey(20) == 27:
                break   
    if VERBOSE:
        cv2.destroyAllWindows()