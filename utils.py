import math
import numpy as np

import numpy as np
from scipy import signal
from ahrs.common.orientation import q_prod, q_conj, acc2q, am2q, q2R, q_rot

def quaternion_to_vector(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z


def update_position(acc, vel, pos, Hz):
    threshold = 0.1
    dt = 1/Hz
    
    sample_number = np.shape(acc)[0]

    t_start = 0
    for t in range(sample_number):
        at = acc[t]
        if np.linalg.norm(at) > threshold:
            t_start = t
            break

    t_end = 0
    for t in range(sample_number - 1, -1, -1):
        at = acc[t]
        if np.linalg.norm(at - acc[-1]) > threshold:
            t_end = t
            break

    an_drift = acc[t_end:].mean(axis=0)
    if t_start != t_end: an_drift_rate = an_drift / (t_end - t_start)

    for i in range(t_end - t_start):
        acc[t_start + i] -= (i + 1) * an_drift_rate

    for i in range(sample_number - t_end):
        acc[t_end + i] -= an_drift

    
    velocities = []
    prevt = -1
    still_phase = False

    v = vel[0]
    t1 = 0
    while t1 < sample_number:
        at = acc[t1, np.newaxis].T

        if np.linalg.norm(at) < threshold:
            if not still_phase:
                predict_v = v + at * dt

                v_drift_rate = predict_v / (t1 - prevt)
                for i in range(t1 - prevt - 1):
                    velocities[prevt + 1 + i] -= (i + 1) * v_drift_rate.T[0]

                v = np.zeros((3, 1))
                prevt = t1
                still_phase = True
        else:
            v = v + at * dt
            still_phase = False

        velocities.append(v.T[0])
        t1 += 1

    velocities = np.array(velocities)

    positions = []
    p = pos[0].T

    t2 = 0
    while t2 < sample_number:
        at = acc[t2, np.newaxis].T
        vt = velocities[t2, np.newaxis].T

        p = p + vt * dt + 0.5 * at * dt**2
        positions.append(p.T[0])
        t2 += 1

    positions = np.array(positions)
    print(positions[-1])
    return velocities[-1], positions[-1]