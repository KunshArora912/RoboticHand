# pc_control.py
# Webcam -> MediaPipe Hands -> 6 servo angles (5 fingers + wrist) -> Arduino (CSV over serial)

import time, math
import serial
import cv2
import mediapipe as mp

# ==== USER SETTINGS ====
COM_PORT = "COM5"      # <-- change to your Arduino port
BAUD     = 115200
SEND_HZ  = 30
MIRROR   = True

# Finger servo limits (min=open, max=closed)
SERVO_OPEN   = 20
SERVO_CLOSED = 160

# Wrist servo limits
WRIST_MIN_DEG = 30     # wrist fully left
WRIST_MAX_DEG = 150    # wrist fully right
WRIST_RANGE_DEG = 80   # how much real rotation to map (~±40° about reference)

# Smoothing
SMOOTH_ALPHA = 0.35
smoothed = [90, 90, 90, 90, 90, 90]
# =======================

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

def openness_to_angle(x):
    return int(round(SERVO_OPEN + x * (SERVO_CLOSED - SERVO_OPEN)))

def hand_to_openness(lm):
    # lm: 21 landmarks, normalized coordinates (x right, y down)
    def finger_metric(tip, pip):  # negative when extended (tip above PIP)
        return lm[tip].y - lm[pip].y

    def lin01(val, v_open=-0.18, v_closed=0.05):
        t = (val - v_open) / (v_closed - v_open)
        return clamp(t, 0.0, 1.0)

    # Thumb: use horizontal spread between tip (4) and MCP (2)
    thumb_spread = abs(lm[4].x - lm[2].x)
    thumb_closed, thumb_open = 0.02, 0.10
    thumb_open01 = 1.0 - clamp((thumb_spread - thumb_closed) / (thumb_open - thumb_closed), 0.0, 1.0)

    idx_open01 = lin01(finger_metric(8, 6))
    mid_open01 = lin01(finger_metric(12, 10))
    rng_open01 = lin01(finger_metric(16, 14))
    pky_open01 = lin01(finger_metric(20, 18))

    return [thumb_open01, idx_open01, mid_open01, rng_open01, pky_open01]

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

            # 5 fingers
            open01 = hand_to_openness(lm)
            raw_fingers = [openness_to_angle(v) for v in open01]

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
