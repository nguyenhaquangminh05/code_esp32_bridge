#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

// ================== PIN MAP ==================
// Motor 1
static const int PIN_EN1   = 19;  // active-low, sẽ giữ LOW luôn
static const int PIN_ZF1   = 18;
static const int PIN_DR1   = 5;
static const int PIN_SP1_1 = 32;
static const int PIN_SP2_1 = 33;

// Motor 2
static const int PIN_EN2   = 23;  // active-low, sẽ giữ LOW luôn
static const int PIN_ZF2   = 22;
static const int PIN_DR2   = 17;
static const int PIN_SP1_2 = 25;
static const int PIN_SP2_2 = 26;

// ================== CONFIG ==================
static const float MAX_RPM = 960.0f;
static const float RPM_DEADBAND = 1.0f;

// Invert direction per motor (CÁCH 1)
static const bool INVERT_DIR_1 = false;
static const bool INVERT_DIR_2 = true;   // <-- thường cần đảo bánh phải

// PWM duty control (tần số cố định)
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;              // 0..255
static const int DUTY_MAX = (1 << PWM_RES) - 1;
static const int CH1 = 0;
static const int CH2 = 1;

// Encoder -> rpm
static const float TICKS_PER_REV = 1400.0f;
static const uint32_t LOG_DT_MS = 20;       // 50 Hz
static const uint32_t CMD_TIMEOUT_MS = 500;

// (Không dùng trong firmware này, nhưng bạn yêu cầu ghi lại)
static const float WHEEL_DIAMETER = 0.162f; // m
static const float WHEEL_DISTANCE = 0.37f;  // m

// ================== ENCODER ==================
ESP32Encoder enc1;
ESP32Encoder enc2;

// ================== STATE ==================
static float rpm_cmd_1 = 0.0f;
static float rpm_cmd_2 = 0.0f;
static uint32_t last_cmd_ms = 0;
static uint32_t last_log_ms = 0;
static int64_t prev_ticks_1 = 0;
static int64_t prev_ticks_2 = 0;

// ================== HELPERS ==================
static inline float clamp_rpm(float rpm) {
  if (rpm >  MAX_RPM) return  MAX_RPM;
  if (rpm < -MAX_RPM) return -MAX_RPM;
  return rpm;
}

// Parse "r a b"
static bool parse_rpm_cmd(const String &line, float &a, float &b) {
  if (line.length() < 2 || line.charAt(0) != 'r') return false;
  int s1 = line.indexOf(' ');
  if (s1 < 0) return false;
  int s2 = line.indexOf(' ', s1 + 1);
  if (s2 < 0) return false;
  a = line.substring(s1 + 1, s2).toFloat();
  b = line.substring(s2 + 1).toFloat();
  return true;
}

// set direction with optional invert
static inline void setDir(int pinZF, float rpm, bool invert) {
  bool dir = (rpm >= 0.0f);
  if (invert) dir = !dir;
  digitalWrite(pinZF, dir ? HIGH : LOW);
}

static void applyMotorDuty(float rpm, int pinZF, int pinDR, int channel, bool invert_dir) {
  rpm = clamp_rpm(rpm);

  if (fabsf(rpm) < RPM_DEADBAND) {
    // Luôn enable (EN giữ LOW), nên stop bằng duty=0
    ledcWrite(channel, 0);
    digitalWrite(pinDR, LOW);
    return;
  }

  setDir(pinZF, rpm, invert_dir);

  float mag = fabsf(rpm) / MAX_RPM; // 0..1
  if (mag > 1.0f) mag = 1.0f;

  int duty = (int)lroundf(mag * DUTY_MAX);
  ledcWrite(channel, duty);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_ZF1, OUTPUT);
  pinMode(PIN_DR1, OUTPUT);

  pinMode(PIN_EN2, OUTPUT);
  pinMode(PIN_ZF2, OUTPUT);
  pinMode(PIN_DR2, OUTPUT);

  // ===== CÁCH A: LUÔN ENABLE ACTIVE-LOW =====
  digitalWrite(PIN_EN1, LOW);   // enable
  digitalWrite(PIN_EN2, LOW);   // enable

  // DR safe
  digitalWrite(PIN_DR1, LOW);
  digitalWrite(PIN_DR2, LOW);

  // PWM channels
  ledcSetup(CH1, PWM_FREQ, PWM_RES);
  ledcSetup(CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_DR1, CH1);
  ledcAttachPin(PIN_DR2, CH2);
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);

  // Encoders
  // ESP32Encoder::useInternalWeakPullResistors = UP; // bật nếu bạn không có pull-up ngoài
  enc1.attachHalfQuad(PIN_SP1_1, PIN_SP2_1);
  enc2.attachHalfQuad(PIN_SP1_2, PIN_SP2_2);
  enc1.clearCount();
  enc2.clearCount();

  prev_ticks_1 = enc1.getCount();
  prev_ticks_2 = enc2.getCount();

  last_cmd_ms = millis();
  last_log_ms = millis();

  Serial.println("Ready.");
  Serial.println("Expect: r rpmL rpmR");
  Serial.println("Publish ticks: e ticksL ticksR");
  Serial.println("Debug: v_set rpmL rpmR | v_fb rpmL rpmR");
}

// ================== LOOP ==================
void loop() {
  // Read command
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    float a, b;
    if (parse_rpm_cmd(line, a, b)) {
      rpm_cmd_1 = clamp_rpm(a);
      rpm_cmd_2 = clamp_rpm(b);
      last_cmd_ms = millis();
    }
  }

  uint32_t now = millis();

  // Timeout -> stop (EN vẫn LOW)
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    rpm_cmd_1 = 0.0f;
    rpm_cmd_2 = 0.0f;
  }

  // Apply motor control (duty) + invert DIR per motor
  applyMotorDuty(rpm_cmd_1, PIN_ZF1, PIN_DR1, CH1, INVERT_DIR_1);
  applyMotorDuty(rpm_cmd_2, PIN_ZF2, PIN_DR2, CH2, INVERT_DIR_2);

  // Publish ticks + print v_set / v_fb
  if (now - last_log_ms >= LOG_DT_MS) {
    float dt = (now - last_log_ms) / 1000.0f;
    last_log_ms = now;

    int64_t t1 = enc1.getCount();
    int64_t t2 = enc2.getCount();
    int64_t d1 = t1 - prev_ticks_1;
    int64_t d2 = t2 - prev_ticks_2;
    prev_ticks_1 = t1;
    prev_ticks_2 = t2;

    float rpm_fb_1 = (dt > 0.0f) ? ((float)d1 * 60.0f / (TICKS_PER_REV * dt)) : 0.0f;
    float rpm_fb_2 = (dt > 0.0f) ? ((float)d2 * 60.0f / (TICKS_PER_REV * dt)) : 0.0f;

    // ticks (NUC parse)
    Serial.print("e ");
    Serial.print((long)t1);
    Serial.print(" ");
    Serial.println((long)t2);

    // debug
    Serial.print("v_set ");
    Serial.print(rpm_cmd_1, 1);
    Serial.print(" ");
    Serial.print(rpm_cmd_2, 1);
    Serial.print(" | v_fb ");
    Serial.print(rpm_fb_1, 1);
    Serial.print(" ");
    Serial.println(rpm_fb_2, 1);
  }
}