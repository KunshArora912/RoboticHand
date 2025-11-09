# pc_control.py
# Webcam -> MediaPipe Hands -> 6 servo angles (5 fingers + wrist) -> Arduino (CSV over serial)

import time, math
import serial
import cv2
import mediapipe as mp

# ==== USER SETTINGS (replace your existing block with this) ====
COM_PORT = "COM5"
BAUD     = 115200
SEND_HZ  = 30
MIRROR   = True

# Per-finger angle limits (thumb, index, middle, ring, pinky)
# Set OPEN < CLOSED. If a finger moves backwards, either swap these two values
# for that finger OR set INVERT_FINGER[i] = True below (software invert).
SERVO_OPEN  = [20, 20, 20, 20, 20]    # open angles per finger
SERVO_CLOSE = [160,160,160,160,160]   # closed angles per finger

# Software invert per finger (thumb..pinky). True means flip 0↔1 for that finger.
INVERT_FINGER = [True, True, True, True, True]   # <- try True for all to start

# Wrist limits
WRIST_MIN_DEG = 30
WRIST_MAX_DEG = 150
WRIST_RANGE_DEG = 80

SMOOTH_ALPHA = 0.35
smoothed = [90, 90, 90, 90, 90, 90]
# ===============================================================


mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6,
    model_complexity=0,
)
drawer = mp.solutions.drawing_utils

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def openness_to_angle(x, i):
    # x in [0..1] (0=open, 1=closed for logic)
    # flip if requested
    if INVERT_FINGER[i]:
        x = 1.0 - x
    lo, hi = SERVO_OPEN[i], SERVO_CLOSE[i]
    return int(round(lo + x * (hi - lo)))


def hand_to_openness(lm):
    """
    Returns 5 openness values in [0..1] for (thumb, index, middle, ring, pinky),
    where 0=open, 1=closed (logical).
    """

    def unit(vx, vy):
        n = (vx*vx + vy*vy) ** 0.5
        return (0.0, 0.0) if n == 0 else (vx/n, vy/n)

    def angle_cos(ax, ay, bx, by):
        ux, uy = unit(ax, ay)
        vx, vy = unit(bx, by)
        return clamp(ux*vx + uy*vy, -1.0, 1.0)  # cos(theta)

    # Generic finger curl using two segments: (MCP->PIP) and (PIP->TIP).
    # cos=1 → straight (open), cos=-1 → fully folded (closed).
    def finger_curl(mcp_idx, pip_idx, tip_idx):
        ax = lm[pip_idx].x - lm[mcp_idx].x
        ay = lm[pip_idx].y - lm[mcp_idx].y
        bx = lm[tip_idx].x - lm[pip_idx].x
        by = lm[tip_idx].y - lm[pip_idx].y
        c = angle_cos(ax, ay, bx, by)  # -1..1
        # Map cos to curl in [0..1]: open(=straight) -> 0, folded -> 1
        # Tunable thresholds for your framing
        c_open, c_closed = 0.85, -0.2
        t = (c_open - c) / (c_open - c_closed)
        return clamp(t, 0.0, 1.0)

    # Thumb: use spread plus a small curl component
    # You can tune these if your thumb behaves oddly.
    thumb_spread = abs(lm[4].x - lm[2].x)
    thumb_closed, thumb_open = 0.02, 0.10
    thumb_spread_open = 1.0 - clamp((thumb_spread - thumb_closed) / (thumb_open - thumb_closed), 0.0, 1.0)
    # Add thumb curl (using CMC approx 2->3 and 3->4)
    thumb_curl = finger_curl(2, 3, 4)
    thumb = clamp(0.6*thumb_spread_open + 0.4*thumb_curl, 0.0, 1.0)

    index  = finger_curl(5, 6, 8)
    middle = finger_curl(9,10,12)
    ring   = finger_curl(13,14,16)
    pinky  = finger_curl(17,18,20)

    return [thumb, index, middle, ring, pinky]


# --- Wrist rotation estimator ---
# We use the vector across knuckles: index MCP (5) -> pinky MCP (17).
# atan2(dy, dx) gives a "roll-ish" angle (radians). We center it at a reference pose.
WRIST_REF_RAD = None  # set on first detection or when you press 'r'

def wrist_angle_deg(lm):
    global WRIST_REF_RAD
    p5, p17 = lm[5], lm[17]
    dx = (p17.x - p5.x)
    dy = (p17.y - p5.y)  # image y is down, but we only need relative angle
    ang = math.atan2(dy, dx)  # -pi..pi

    if WRIST_REF_RAD is None:
        WRIST_REF_RAD = ang

    delta = ang - WRIST_REF_RAD
    # normalize to [-pi, pi]
    while delta <= -math.pi: delta += 2*math.pi
    while delta >   math.pi: delta -= 2*math.pi

    # Map ±WRIST_RANGE_DEG around reference → [WRIST_MIN_DEG .. WRIST_MAX_DEG]
    span_rad = math.radians(WRIST_RANGE_DEG)
    t = clamp((delta + span_rad) / (2*span_rad), 0.0, 1.0)
    deg = int(round(WRIST_MIN_DEG + t * (WRIST_MAX_DEG - WRIST_MIN_DEG)))
    return deg

def open_serial(port, baud):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=0)
            time.sleep(2)
            return ser
        except Exception as e:
            print(f"[INFO] Waiting for {port} @ {baud}... ({e})")
            time.sleep(1)

ser = open_serial(COM_PORT, BAUD)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Webcam not found. Try index 1/2.")

send_interval = 1.0 / SEND_HZ
t_last = 0.0

print("[OK] Running. Keys: 'q' quit, 'r' reset wrist reference.")
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if MIRROR:
            frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        angles = None
        if res.multi_hand_landmarks:
            hand = res.multi_hand_landmarks[0]
            lm = hand.landmark

            open01 = hand_to_openness(lm)              # list of 5 floats 0..1
            raw_fingers = [openness_to_angle(open01[i], i) for i in range(5)]


            # wrist
            raw_wrist = wrist_angle_deg(lm)

            # smooth
            raw_angles = raw_fingers + [raw_wrist]
            for i in range(6):
                smoothed[i] = int(round(SMOOTH_ALPHA * raw_angles[i] + (1 - SMOOTH_ALPHA) * smoothed[i]))
            angles = smoothed

            # draw landmarks
            mp.solutions.drawing_utils.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            txt = "Angles(T,I,M,R,P,W): " + ",".join(f"{a:3d}" for a in angles)
            cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("MediaPipe Hand -> Servos (6ch)", frame)

        # keyboard
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('r'):
            WRIST_REF_RAD = None  # next frame will recenter wrist

        # send
        now = time.time()
        if angles and (now - t_last) >= send_interval:
            packet = ",".join(str(a) for a in angles) + "\n"
            try:
                ser.write(packet.encode("ascii"))
            except Exception as e:
                print(f"[WARN] Serial write failed: {e}. Reopening...")
                try:
                    ser.close()
                except:
                    pass
                ser = open_serial(COM_PORT, BAUD)
            t_last = now

finally:
    cap.release()
    ser.close()
    cv2.destroyAllWindows()
