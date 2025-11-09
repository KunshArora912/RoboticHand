// arduino_servos_6.ino
// Receives "a0,a1,a2,a3,a4,a5\n" (thumb,index,middle,ring,pinky,wrist)
// and drives 6 servos with easing.

#include <Servo.h>

static const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11}; // last = wrist
Servo servos[6];

int current[6] = {90, 90, 90, 90, 90, 90};
int target [6] = {90, 90, 90, 90, 90, 90};

const int STEP = 3;            // deg per loop
const int LOOP_DELAY_MS = 10;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; ++i) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(current[i]);
  }
}

bool parseLineToAngles(const String& line, int out[], int count) {
  int start = 0;
  for (int i = 0; i < count; ++i) {
    int comma = (i < count - 1) ? line.indexOf(',', start) : line.length();
    if (comma < 0) return false;
    String tok = line.substring(start, comma);
    tok.trim();
    if (tok.length() == 0) return false;
    out[i] = constrain(tok.toInt(), 0, 180);
    start = comma + 1;
  }
  return true;
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int vals[6];
      if (parseLineToAngles(line, vals, 6)) {
        for (int i = 0; i < 6; ++i) target[i] = vals[i];
      }
    }
  }

  bool changed = false;
  for (int i = 0; i < 6; ++i) {
    if (current[i] < target[i]) {
      current[i] = min(current[i] + STEP, target[i]);
      changed = true;
    } else if (current[i] > target[i]) {
      current[i] = max(current[i] - STEP, target[i]);
      changed = true;
    }
    servos[i].write(current[i]);
  }

  delay(LOOP_DELAY_MS);
}
