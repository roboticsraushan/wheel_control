/*
  wheel_control_mega.ino

  Arduino Mega 2560 + Cytron dual channel driver (DIR+PWM) control.
  Pins:
    - Motor 1: DIR -> D7,  PWM -> D8 (PWM)
    - Motor 2: DIR -> D9,  PWM -> D10 (PWM)

  Serial protocol (115200 baud):
    - Set motor speeds:  M <left> <right>\n
      where <left>, <right> are integers in range [-255, 255]
      Example: "M -120 200\n"  => Left reverse 120, Right forward 200

    - Stop both motors immediately: S\n
    - Ping: P\n  Responses (for debugging):
    - OK, ERR, PONG and telemetry lines starting with '#'

  Safety:
    - Watchdog stops both motors if no command received for WATCHDOG_MS.
*/

#include <Arduino.h>

// ========================= Config =========================
const uint8_t DIR1_PIN = 7;   // Motor 1 direction
const uint8_t PWM1_PIN = 8;   // Motor 1 PWM (must be PWM-capable)
const uint8_t DIR2_PIN = 9;   // Motor 2 direction
const uint8_t PWM2_PIN = 10;  // Motor 2 PWM (must be PWM-capable)

// Invert direction per motor if wiring results in opposite motion
const bool INVERT_M1 = false;   // Left motor inverted
const bool INVERT_M2 = true;

// Serial settings
const unsigned long SERIAL_BAUD = 115200;

// Watchdog timeout (ms). If no valid command within this time, stop motors.
const unsigned long WATCHDOG_MS = 500;  // adjust as needed

// Uncomment to enable periodic telemetry prints (every TELEMETRY_MS)
//#define ENABLE_TELEMETRY 1
const unsigned long TELEMETRY_MS = 1000;

// ========================= Globals ========================
char rx_buf[64];               // input line buffer
size_t rx_len = 0;             // current buffer length
unsigned long last_cmd_ms = 0; // last time a valid command was received
unsigned long last_telemetry_ms = 0;

int16_t last_left = 0;
int16_t last_right = 0;

// ===================== Motor helpers ======================
inline void setMotorRaw(uint8_t dirPin, uint8_t pwmPin, bool invert, int16_t speed) {
  // constrain to [-255, 255]
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  bool forward = (speed >= 0);
  uint8_t duty = (uint8_t)abs(speed);

  if (invert) forward = !forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, duty);
}

inline void setMotors(int16_t left, int16_t right) {
  last_left = left;
  last_right = right;
  setMotorRaw(DIR1_PIN, PWM1_PIN, INVERT_M1, left);
  setMotorRaw(DIR2_PIN, PWM2_PIN, INVERT_M2, right);
}

inline void stopMotors() {
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
}

// ===================== Serial parsing =====================
void handleLine(char* line) {
  // Trim trailing CR/LF
  size_t n = strlen(line);
  while (n > 0 && (line[n-1] == '\r' || line[n-1] == '\n' || line[n-1] == ' ')) {
    line[--n] = '\0';
  }
  // Skip leading spaces
  while (*line == ' ') line++;

  if (n == 0 || line[0] == '\0') return;

  switch (line[0]) {
    case 'M':
    case 'm': {
      long l = 0, r = 0;
      // Accept formats: "M L R", "M L,R"
      int parsed = sscanf(line + 1, "%ld%*[ ,\t]%ld", &l, &r);
      if (parsed == 2) {
        if (l < -255) l = -255; if (l > 255) l = 255;
        if (r < -255) r = -255; if (r > 255) r = 255;
        setMotors((int16_t)l, (int16_t)r);
        last_cmd_ms = millis();
        Serial.print(F("OK L:"));
        Serial.print(l);
        Serial.print(F(" R:"));
        Serial.println(r);
      } else {
        Serial.println(F("ERR bad M"));
      }
      break;
    }
    case 'S':
    case 's': {
      stopMotors();
      last_left = 0; last_right = 0;
      Serial.println(F("OK"));
      break;
    }
    case 'P':
    case 'p': {
      Serial.println(F("PONG"));
      break;
    }
    default:
      Serial.println(F("ERR unknown"));
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // terminate and handle
      rx_buf[rx_len] = '\0';
      handleLine(rx_buf);
      rx_len = 0; // reset
    } else if (c == '\r') {
      // ignore
    } else {
      if (rx_len < sizeof(rx_buf) - 1) {
        rx_buf[rx_len++] = c;
      } else {
        // overflow, reset buffer
        rx_len = 0;
        Serial.println(F("ERR overflow"));
      }
    }
  }
}

// ======================== Arduino ========================
void setup() {
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  stopMotors();

  Serial.begin(SERIAL_BAUD);
  // Arduino Due doesn't need to wait for Serial
  delay(1000);  // Give time for serial to initialize

  Serial.println(F("# wheel_control_mega ready"));
  Serial.print(F("# Pins DIR1=")); Serial.print(DIR1_PIN);
  Serial.print(F(" PWM1=")); Serial.print(PWM1_PIN);
  Serial.print(F(" DIR2=")); Serial.print(DIR2_PIN);
  Serial.print(F(" PWM2=")); Serial.println(PWM2_PIN);

  last_cmd_ms = millis();
  last_telemetry_ms = millis();
}

void loop() {
  pollSerial();

  // Watchdog: stop if timed out
  unsigned long now = millis();
  if (now - last_cmd_ms > WATCHDOG_MS) {
    if (last_left != 0 || last_right != 0) {
      stopMotors();
      last_left = 0; last_right = 0;
      Serial.println(F("# watchdog stop"));
    }
    last_cmd_ms = now; // prevent repeated prints
  }

#ifdef ENABLE_TELEMETRY
  if (now - last_telemetry_ms >= TELEMETRY_MS) {
    last_telemetry_ms = now;
    Serial.print(F("# L:")); Serial.print(last_left);
    Serial.print(F(" R:")); Serial.println(last_right);
  }
#endif
}
