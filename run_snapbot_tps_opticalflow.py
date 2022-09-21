import cv2
import os
import numpy as np
import time
import math
import apriltag
import pyrealsense2 as rs
from realworld_func.class_motionhelper import timer
from realworld_func.class_xm430 import xm430
from utils_track import tps_trans

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


lk_params = dict(winSize  = (15, 15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict(maxCorners = 10,
                    qualityLevel = 0.3,
                    minDistance = 10,
                    blockSize = 7 )

def run_snapbot_tps_opticalflow(qpos, tps_coef, snapbot, Hz, max_sec, idx, VERBOSE=False):
    traj = get_traj(qpos)

    x = np.linspace(170,470,4)
    y = np.linspace(90,390,4)
    X, Y = np.meshgrid(x,y)
    ctrl_xy = np.stack([X,Y],axis=2).reshape(-1,2)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    trajectory_len = 40
    detect_interval = 5
    trajectories = []
    frame_idx = 0
    cap = cv2.VideoCapture(idx)
    of_score = 0

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
            
            if VERBOSE: 
                cv2.putText(color_image, "({},{}),{}".format(cX, cY, deg), (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if VERBOSE: 
            cv2.imshow("Frame", color_image)
            if cv2.waitKey(20) == 27:
                break   

        start = time.time()
        suc, frame = cap.read()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = frame.copy()
        img = Rotate(img,180) # Rotate 

        # Calculate optical flow for a sparse feature set using the iterative Lucas-Kanade Method
        if len(trajectories) > 0:
            img0, img1 = prev_gray, frame_gray
            p0 = np.float32([trajectory[-1] for trajectory in trajectories]).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1

            new_trajectories = []

            # Get all the trajectories
            for trajectory, (x, y), good_flag in zip(trajectories, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                trajectory.append((x, y))
                if len(trajectory) > trajectory_len:
                    del trajectory[0]
                new_trajectories.append(trajectory)
                # Newest detected point
                cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), -1)

            trajectories = new_trajectories

            # Draw all the trajectories
            cv2.polylines(img, [np.int32(trajectory) for trajectory in trajectories], False, (0, 255, 0))
            cv2.putText(img, 'track count: %d' % len(trajectories), (20, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)

            left_sc = 0
            go_sc = 0

            for trajectory in trajectories:
                xzero = trajectory[0][0]
                yzero = trajectory[0][1]
                xprime = trajectory[-1][0]
                yprime = trajectory[-1][1]
                left_sc += xprime - xzero
                go_sc += yprime - yzero
            of_score += left_sc
            
        # Update interval - When to update and detect new features
        if frame_idx % detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255

            # Lastest point in latest trajectory
            for x, y in [np.int32(trajectory[-1]) for trajectory in trajectories]:
                cv2.circle(mask, (x, y), 5, 0, -1)

            # Detect the good features to track
            p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
            if p is not None:
                # If good features can be tracked - add that to the trajectories
                for x, y in np.float32(p).reshape(-1, 2):
                    trajectories.append([(x, y)])

        frame_idx += 1
        prev_gray = frame_gray

        # End time
        end = time.time()
        # calculate the FPS for current frame detection
        fps = 1 / (end-start)
        
        # Show Results
        cv2.putText(img, f"{fps:.2f} FPS", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Optical Flow', img)
        cv2.imshow('Mask', mask)
        # cv2.waitKey(0)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            pass 

    print(of_score/10000)

    cap.release()
    cv2.destroyAllWindows()