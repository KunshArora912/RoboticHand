# servo_step_tester.py  (letter keys version)
import time
import serial
import cv2
import numpy as np

# ====== USER SETTINGS ======
COM_PORT = "COM5"    # <-- set your Arduino COM port
BAUD     = 115200
INIT_ANGLES = [90, 90, 90, 90, 90, 90]   # thumb..wrist start
UI_W, UI_H = 1000, 380
# ===========================

LABELS = ["Thumb","Index","Middle","Ring","Pinky","Wrist"]

def open_serial(port, baud):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=0)
            time.sleep(2)  # let Arduino reset
            print(f"[OK] Connected {port} @ {baud}")
            return ser
        except Exception as e:
            print(f"[INFO] Waiting for {port} @ {baud}... ({e})")
            time.sleep(1)

def clamp(v, lo=-100, hi=200):
    return lo if v < lo else hi if v > hi else v

def send_angles(ser, angles):
    pkt = ",".join(str(int(a)) for a in angles) + "\n"
    try:
        ser.write(pkt.encode("ascii"))
    except Exception as e:
        print(f"[WARN] Serial write failed: {e}")

def draw_ui(angles, selected):
    img = np.zeros((UI_H, UI_W, 3), dtype=np.uint8)
    y = 40; pad = 20
    cv2.putText(img, "Servo Step Tester (6ch)", (pad, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2); y += 30
    cv2.putText(img,
        "1-6 select | w/s +/-1 | e/d +/-5 | Space=90 | a all=sel | r reset others | t sweep | q quit",
        (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1); y += 30
    for i, name in enumerate(LABELS):
        color = (0,255,255) if i == selected else (220,220,220)
        txt = f"{i+1}. {name:6s} : {angles[i]:3d} deg"
        cv2.putText(img, txt, (pad, y + i*28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return img

def sweep_selected(ser, angles, idx):
    # sweep 0->200->0 while updating UI
    for seq in [range(-100,181,2), range(200,-1,-2)]:
        for a in seq:
            angles[idx] = a
            send_angles(ser, angles)
            # quick visual heartbeat in console
            print(f"[SWEEP] {LABELS[idx]} = {a:3d}", end="\r")
            time.sleep(0.01)
    print()

def main():
    ser = open_serial(COM_PORT, BAUD)
    angles = INIT_ANGLES[:]
    selected = 0
    send_angles(ser, angles)

    cv2.namedWindow("Servo Tester", cv2.WINDOW_AUTOSIZE)
    print("[OK] Controls ready. Make sure Arduino Serial Monitor is CLOSED.")
    try:
        while True:
            img = draw_ui(angles, selected)
            cv2.imshow("Servo Tester", img)
            key = cv2.waitKey(50) & 0xFF  # ASCII-friendly

            changed = False
            if key == ord('q'):
                break

            # Select channel 1..6
            if key in (ord('1'),ord('2'),ord('3'),ord('4'),ord('5'),ord('6')):
                selected = key - ord('1')
                print(f"[SEL] {LABELS[selected]} ({selected+1})")

            # Fine steps
            elif key == ord('w'):   # +1
                angles[selected] = clamp(angles[selected] + 1); changed = True
            elif key == ord('s'):   # -1
                angles[selected] = clamp(angles[selected] - 1); changed = True

            # Coarse steps
            elif key == ord('e'):   # +5
                angles[selected] = clamp(angles[selected] + 5); changed = True
            elif key == ord('d'):   # -5
                angles[selected] = clamp(angles[selected] - 5); changed = True

            # Center selected
            elif key == ord(' '):
                angles[selected] = 90; changed = True

            # Set all = selected angle
            elif key == ord('a'):
                angles = [angles[selected]] * 6; changed = True

            # Reset others to 90
            elif key == ord('r'):
                for i in range(6):
                    if i != selected:
                        angles[i] = 90
                changed = True

            # Sweep test
            elif key == ord('t'):
                sweep_selected(ser, angles, selected)
                changed = True

            if changed:
                print(f"[SET] {LABELS[selected]} -> {angles[selected]} | All: {angles}")
                send_angles(ser, angles)

    finally:
        try: ser.close()
        except: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
