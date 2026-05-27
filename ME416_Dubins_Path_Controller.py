#!/usr/bin/env python3

import socket
import time
import signal
import sys
import threading
import math
import json

import numpy as np
import paho.mqtt.client as mqtt

# =======================
# Config
# =======================

MQTT_BROKER = "rasticvm.lan"
MQTT_PORT = 1883

LIMO_ID = "limo777"
MQTT_TOPIC_WILDCARD = f"rb/{LIMO_ID}/#"

# LIMO TCP server
LIMO_IP = "192.168.1.101"
LIMO_PORT = 12345

# Path file exported from MATLAB (all courses)
ALL_PATHS_FILE = "all_paths.csv"

# Choose which course to follow: 1, 2, or 3
COURSE_ID = 2          # <-- change this for course 1/2/3

# Toggle between real CSV path and debug L-path
USE_DEBUG_L_PATH = False   # set True if you want the simple L again

# Control parameters
V_FF = 0.35
V_MAX = 1.0
OMEGA_MAX = 1.0

KX = 0.0
KY = 0.40
KTH = 0.60

GOAL_TOL = 0.15
DT_MIN   = 0.05

LAT_DEAD  = 0.08
YAW_DEAD  = 0.15
W_SMALL   = 0.15

W_FILTER_ALPHA = 0.2  # 0.2 = smooth, 1.0 = no filter

# Yaw offset between OptiTrack frame and "robot forward"
# This MUST match what worked in your visualizer.
YAW_OFFSET = math.pi / 2  # if this is wrong now, flip to -math.pi/2


# =======================
# Pose state
# =======================

class PoseState:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = float("nan")   # arena X
        self.y = float("nan")   # arena Y (we store arena Z here)
        self.yaw = float("nan")

    def update_pos(self, x, y):
        with self.lock:
            self.x = x
            self.y = y

    def update_yaw(self, yaw):
        with self.lock:
            self.yaw = yaw

    def get(self):
        with self.lock:
            return self.x, self.y, self.yaw

pose_state = PoseState()


# =======================
# Utility functions
# =======================

def angle_wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def quat_to_yaw(qx, qy, qz, qw):
    """
    OptiTrack:
      pos = [x, y, z]
      rot = [qx, qy, qz, qw]

    Y is up, robot moves in X–Z plane.
    We compute yaw by rotating body +Z into world and projecting onto X–Z,
    then apply a constant offset (YAW_OFFSET) to line up with the robot's
    real forward direction.
    """
    # reorder → (w, x, y, z)
    w, x, y, z = qw, qx, qy, qz

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ])

    forward_world = R @ np.array([0.0, 0.0, 1.0])  # body +Z
    fx = forward_world[0]
    fz = forward_world[2]

    yaw_raw = math.atan2(fx, fz)
    yaw = yaw_raw + YAW_OFFSET
    yaw = angle_wrap_pi(yaw)
    return yaw


# =======================
# Paths
# =======================

def load_course_path(filename, course_id):
    """
    CSV rows: [course_id, x, y, yaw]
    Returns Nx3 [x, y, yaw] for given course.
    """
    data = np.loadtxt(filename, delimiter=",")
    mask = (data[:, 0].astype(int) == course_id)
    path = data[mask, 1:4]

    if path.shape[0] == 0:
        raise RuntimeError(f"No path with course_id={course_id}")

    print(f"[PATH] Loaded {path.shape[0]} points for course {course_id}")
    return path


def make_debug_L_path():
    """
    Simple L-shaped path in its own frame:

      1. Straight forward 1.5 m
      2. Right turn 90° and 1.5 m
    """
    L1 = 1.5
    L2 = 1.5
    N1 = 60
    N2 = 60

    x1 = np.linspace(0.0, L1, N1)
    y1 = np.zeros(N1)

    x2 = np.full(N2, L1)
    y2 = np.linspace(0.0, -L2, N2)

    x = np.concatenate([x1, x2])
    y = np.concatenate([y1, y2])
    yaw = np.zeros_like(x)

    path = np.column_stack((x, y, yaw))
    print(f"[DEBUG PATH] Built L-shape: N={path.shape[0]} points")
    return path


# =======================
# MQTT callbacks
# =======================

def on_connect(client, userdata, flags, rc, props=None):
    print("[MQTT] Connected.")
    client.subscribe(MQTT_TOPIC_WILDCARD)
    print(f"[MQTT] Subscribed to {MQTT_TOPIC_WILDCARD}")

def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8").strip()

    # Try to parse JSON
    try:
        data = json.loads(payload)
    except Exception:
        return

    # Only use messages with pos + rot
    if not (isinstance(data, dict) and "pos" in data and "rot" in data):
        return

    pos = data["pos"]
    rot = data["rot"]

    if len(pos) >= 3 and len(rot) == 4:
        x = float(pos[0])        # mocap X
        z = float(pos[2])        # mocap Z (floor)
        pose_state.update_pos(x, z)

        qx, qy, qz, qw = map(float, rot)
        yaw = quat_to_yaw(qx, qy, qz, qw)
        pose_state.update_yaw(yaw)
        # print(f"[MOCAP] x={x:.3f}, z={z:.3f}, yaw={yaw:.3f} rad")


# =======================
# Feedforward omega
# =======================

def compute_omega_ff(path):
    """
    path: Nx3 [x, y, yaw]
    """
    N = path.shape[0]
    omega_ff = np.zeros(N)

    for i in range(N - 1):
        dx = path[i+1, 0] - path[i, 0]
        dy = path[i+1, 1] - path[i, 1]
        ds = math.hypot(dx, dy)
        if ds < 1e-4:
            ds = 1e-3
        dth = angle_wrap_pi(path[i+1, 2] - path[i, 2])
        dt = ds / max(V_FF, 0.05)
        omega_ff[i] = dth / dt

    omega_ff[-1] = omega_ff[-2]
    return omega_ff


# =======================
# RELATIVE / START-ANYWHERE FOLLOWER
# =======================

def follow_path(sock, path_nominal, label="course"):
    """
    Follow the path SHAPE, starting from wherever the robot is.
    """
    # ---------- 0) Recompute heading from (x,y) ----------
    N = path_nominal.shape[0]
    x_nom = path_nominal[:, 0]
    y_nom = path_nominal[:, 1]

    th_nom = np.zeros(N)
    for k in range(N - 1):
        dx = x_nom[k+1] - x_nom[k]
        dy = y_nom[k+1] - y_nom[k]
        th_nom[k] = math.atan2(dy, dx)
    th_nom[-1] = th_nom[-2]

    print(f"[CTRL] {label}: N = {N} points (geom yaw)")
    print(f"[CTRL]   Nom first: x={x_nom[0]:.2f}, y={y_nom[0]:.2f}, yaw={th_nom[0]:.2f}")
    mid = N // 2
    print(f"[CTRL]   Nom mid:   x={x_nom[mid]:.2f}, y={y_nom[mid]:.2f}, yaw={th_nom[mid]:.2f}")
    print(f"[CTRL]   Nom last:  x={x_nom[-1]:.2f}, y={y_nom[-1]:.2f}, yaw={th_nom[-1]:.2f}")

    # ---------- 1) Make path relative to its own first pose ----------
    xr0, yr0, thr0 = x_nom[0], y_nom[0], th_nom[0]

    dx_nom = x_nom - xr0
    dy_nom = y_nom - yr0

    c0 = math.cos(-thr0)
    s0 = math.sin(-thr0)

    x_rel = c0 * dx_nom - s0 * dy_nom
    y_rel = s0 * dx_nom + c0 * dy_nom
    th_rel = np.array([angle_wrap_pi(th - thr0) for th in th_nom])

    # ---------- 2) Wait for robot initial pose ----------
    print("[CTRL] Waiting for valid mocap pose...")
    while True:
        x0, y0, th0 = pose_state.get()
        if not (math.isnan(x0) or math.isnan(th0)):
            break
        print("[CTRL] Pose NaN, still waiting...")
        time.sleep(0.2)

    print(f"[CTRL] Robot initial pose: x={x0:.2f}, y={y0:.2f}, yaw={th0:.2f}")

    # ---------- 3) Anchor relative path to robot start pose ----------
    cR = math.cos(th0)
    sR = math.sin(th0)

    x_anch = x0 + cR * x_rel - sR * y_rel
    y_anch = y0 + sR * x_rel + cR * y_rel
    th_anch = np.array([angle_wrap_pi(th0 + dth) for dth in th_rel])

    path = np.column_stack((x_anch, y_anch, th_anch))

    print("[CTRL] Anchored path:")
    print(f"       First: x={path[0,0]:.2f}, y={path[0,1]:.2f}, yaw={path[0,2]:.2f}")
    print(f"       Last:  x={path[-1,0]:.2f}, y={path[-1,1]:.2f}, yaw={path[-1,2]:.2f}")

    # ---------- 4) Feedforward omega from anchored path ----------
    omega_ff = compute_omega_ff(path)
    goal = path[-1, :2]

    ADVANCE_DIST = 0.08

    i = 0
    prev_w = 0.0

    print(f"[CTRL] Starting START-ANYWHERE tracking for {label}...")

    try:
        while True:
            loop_start = time.time()
            x, y, th = pose_state.get()

            # goal check
            dg = math.hypot(x - goal[0], y - goal[1])
            if dg < GOAL_TOL:
                print(f"[CTRL] GOAL REACHED (dist={dg:.3f})")
                sock.send(b"0.000,0.000")
                break

            if i >= N:
                i = N - 1

            xr, yr, thr = path[i]
            w_ff = omega_ff[i]

            dx = xr - x
            dy = yr - y
            dist_wp = math.hypot(dx, dy)

            # robot-frame errors
            ex = math.cos(th) * dx + math.sin(th) * dy
            ey = -math.sin(th) * dx + math.cos(th) * dy
            eth = angle_wrap_pi(thr - th)

            # forward speed
            v = V_FF + KX * ex
            if v < 0:
                v = 0

            # steering: FEEDFORWARD + FEEDBACK
            w_raw = w_ff + KY * (-ey) + KTH * eth

            # deadzones (keep feedforward alive)
            if abs(ey) < LAT_DEAD and abs(eth) < YAW_DEAD:
                w_raw = w_ff

            if abs(w_raw) < W_SMALL:
                w_raw = 0.0

            # low-pass filter
            w = (1 - W_FILTER_ALPHA) * prev_w + W_FILTER_ALPHA * w_raw
            prev_w = w

            v = clamp(v, 0, V_MAX)
            w = clamp(w, -OMEGA_MAX, OMEGA_MAX)

            cmd = f"{v:.3f},{w:.3f}"
            sock.send(cmd.encode())

            # advance waypoint when close enough
            if dist_wp < ADVANCE_DIST and i < N - 1:
                i += 1

            # loop timing
            time.sleep(max(0, DT_MIN - (time.time() - loop_start)))

    finally:
        try:
            sock.send(b"0.000,0.000")
        except:
            pass
        print("[CTRL] Control loop finished.")


# =======================
# MAIN
# =======================

def main():
    if USE_DEBUG_L_PATH:
        print("[MAIN] Using simple debug L-path...")
        path_nominal = make_debug_L_path()
        label = "debug L-path"
    else:
        print(f"[MAIN] Loading course {COURSE_ID} from {ALL_PATHS_FILE}...")
        path_nominal = load_course_path(ALL_PATHS_FILE, COURSE_ID)
        label = f"course {COURSE_ID}"

    print("[MAIN] Connecting to LIMO...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((LIMO_IP, LIMO_PORT))
    print("[MAIN] LIMO connected.")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print("[MAIN] Connecting to MQTT...")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
    client.loop_start()

    def kill(sig, frame):
        print("[MAIN] Ctrl-C received")
        try:
            sock.send(b"0.000,0.000")
        except:
            pass
        sock.close()
        client.loop_stop()
        client.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, kill)

    # give MQTT a moment to get some pose messages
    time.sleep(1.0)

    follow_path(sock, path_nominal, label=label)

    print("[MAIN] Stopping...")
    try:
        sock.send(b"0.000,0.000")
    except:
        pass
    sock.close()
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()
