#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

// ================== PIN MAP ==================
// Motor 1
static const int PIN_EN1   = 19;  // active-low, giữ LOW luôn
static const int PIN_ZF1   = 18;
static const int PIN_DR1   = 5;
static const int PIN_SP1_1 = 32;
static const int PIN_SP2_1 = 33;

// Motor 2
static const int PIN_EN2   = 23;  // active-low, giữ LOW luôn
static const int PIN_ZF2   = 22;
static const int PIN_DR2   = 17;
static const int PIN_SP1_2 = 25;
static const int PIN_SP2_2 = 26;

// ================== CONFIG ==================
static const float MAX_RPM = 382.71f;
static const float RPM_DEADBAND = 1.0f;

// đảo chiều theo lắp cơ khí
static const bool INVERT_DIR_1 = false;
static const bool INVERT_DIR_2 = true;

// PWM
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int DUTY_MAX = (1 << PWM_RES) - 1; // 255
static const int CH1 = 0;
static const int CH2 = 1;

// Encoder
static const float TICKS_PER_REV = 1400.0f;

// Timing
static const uint32_t CTRL_DT_MS = 20;          // PI update 50Hz
static const uint32_t PUB_DT_MS  = 20;          // publish ticks 50Hz
static const uint32_t CMD_TIMEOUT_MS = 500;

// mapping đo được: 1 duty ≈ 1.5 rpm
static const float RPM_PER_DUTY = 1.5f;

// ===== PI gains =====
static float KP = 0.80f;                        // duty/rpm
static float KI = 1.50f;                        // duty/(rpm*s)

// Anti-windup & stiction
static const float I_LIMIT = 120.0f;            // giới hạn I (duty)
static const int   MIN_DUTY = 20;               // thử 20..60

// ===== Filter + Slew (chống giật) =====
// Lọc rpm feedback (EMA low-pass)
static const float TAU_RPM = 0.12f;             // 0.08~0.20s
static float rpm_f_1 = 0.0f;
static float rpm_f_2 = 0.0f;

// Giới hạn tốc độ thay đổi duty mỗi chu kỳ control
static const float DUTY_SLEW = 6.0f;            // duty/20ms (tune 3..10)
static float duty_prev_1 = 0.0f;
static float duty_prev_2 = 0.0f;

// ================== ENCODER ==================
ESP32Encoder enc1;
ESP32Encoder enc2;

// ================== STATE ==================
static float rpm_cmd_1 = 0.0f, rpm_cmd_2 = 0.0f;
static float rpm_fb_1  = 0.0f, rpm_fb_2  = 0.0f;

static float integ1 = 0.0f, integ2 = 0.0f;
static float duty_out_1 = 0.0f, duty_out_2 = 0.0f;

static uint32_t last_cmd_ms = 0;
static uint32_t last_ctrl_ms = 0;
static uint32_t last_pub_ms  = 0;

static int64_t prev_ticks_1 = 0;
static int64_t prev_ticks_2 = 0;

// ================== HELPERS ==================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float clamp_rpm(float rpm) {
  return clampf(rpm, -MAX_RPM, MAX_RPM);
}

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

static inline void setDir(int pinZF, float rpm_cmd, bool invert) {
  bool dir = (rpm_cmd >= 0.0f);
  if (invert) dir = !dir;
  digitalWrite(pinZF, dir ? HIGH : LOW);
}

static inline float slew(float u, float &u_prev) {
  float du = u - u_prev;
  if (du >  DUTY_SLEW) du =  DUTY_SLEW;
  if (du < -DUTY_SLEW) du = -DUTY_SLEW;
  u_prev += du;
  return u_prev;
}

static void applyDuty(float rpm_cmd, float duty_mag, int pinZF, int pinDR, int channel, bool invert_dir) {
  if (fabsf(rpm_cmd) < RPM_DEADBAND) {
    ledcWrite(channel, 0);
    digitalWrite(pinDR, LOW);
    return;
  }

  setDir(pinZF, rpm_cmd, invert_dir);

  int duty = (int)lroundf(clampf(duty_mag, 0.0f, (float)DUTY_MAX));
  if (duty > 0 && duty < MIN_DUTY) duty = MIN_DUTY;

  ledcWrite(channel, duty);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(PIN_EN1, OUTPUT); pinMode(PIN_ZF1, OUTPUT); pinMode(PIN_DR1, OUTPUT);
  pinMode(PIN_EN2, OUTPUT); pinMode(PIN_ZF2, OUTPUT); pinMode(PIN_DR2, OUTPUT);

  // enable active-low luôn
  digitalWrite(PIN_EN1, LOW);
  digitalWrite(PIN_EN2, LOW);

  // PWM
  ledcSetup(CH1, PWM_FREQ, PWM_RES);
  ledcSetup(CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_DR1, CH1);
  ledcAttachPin(PIN_DR2, CH2);
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);

  // encoders
  enc1.attachHalfQuad(PIN_SP1_1, PIN_SP2_1);
  enc2.attachHalfQuad(PIN_SP1_2, PIN_SP2_2);
  enc1.clearCount();
  enc2.clearCount();

  prev_ticks_1 = enc1.getCount();
  prev_ticks_2 = enc2.getCount();

  uint32_t now = millis();
  last_cmd_ms  = now;
  last_ctrl_ms = now;
  last_pub_ms  = now;
}

// ================== LOOP ==================
void loop() {
  // ---- receive v_set (rpm) anytime ----
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    float a, b;
    if (parse_rpm_cmd(line, a, b)) {
      rpm_cmd_1 = clamp_rpm(a);
      rpm_cmd_2 = clamp_rpm(b);
      last_cmd_ms = millis();

      // reset khi stop để tránh kéo đuôi
      if (fabsf(rpm_cmd_1) < RPM_DEADBAND) { integ1 = 0; duty_out_1 = 0; rpm_f_1 = 0; duty_prev_1 = 0; }
      if (fabsf(rpm_cmd_2) < RPM_DEADBAND) { integ2 = 0; duty_out_2 = 0; rpm_f_2 = 0; duty_prev_2 = 0; }
    }
  }

  uint32_t now = millis();

  // ---- timeout safety ----
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    rpm_cmd_1 = 0.0f;
    rpm_cmd_2 = 0.0f;
  }

  // ---- PI update fixed 50Hz ----
  if (now - last_ctrl_ms >= CTRL_DT_MS) {
    float dt = (now - last_ctrl_ms) / 1000.0f;
    last_ctrl_ms = now;

    // read encoder -> rpm_raw
    int64_t t1 = enc1.getCount();
    int64_t t2 = enc2.getCount();
    int64_t d1 = t1 - prev_ticks_1;
    int64_t d2 = t2 - prev_ticks_2;
    prev_ticks_1 = t1;
    prev_ticks_2 = t2;

    float rpm_raw_1 = (dt > 0.0f) ? ((float)d1 * 60.0f / (TICKS_PER_REV * dt)) : 0.0f;
    float rpm_raw_2 = (dt > 0.0f) ? ((float)d2 * 60.0f / (TICKS_PER_REV * dt)) : 0.0f;

    // EMA low-pass on rpm
    float alpha = (dt > 0.0f) ? (dt / (TAU_RPM + dt)) : 1.0f;
    rpm_f_1 = rpm_f_1 + alpha * (rpm_raw_1 - rpm_f_1);
    rpm_f_2 = rpm_f_2 + alpha * (rpm_raw_2 - rpm_f_2);

    rpm_fb_1 = rpm_f_1;
    rpm_fb_2 = rpm_f_2;

    // PI theo độ lớn để tránh rối dấu do lắp ngược
    float vset1 = fabsf(rpm_cmd_1);
    float vset2 = fabsf(rpm_cmd_2);
    float vfb1  = fabsf(rpm_fb_1);
    float vfb2  = fabsf(rpm_fb_2);

    float e1 = vset1 - vfb1;
    float e2 = vset2 - vfb2;

    // feedforward từ mapping 1.5 rpm/duty
    float duty_ff_1 = vset1 / RPM_PER_DUTY;
    float duty_ff_2 = vset2 / RPM_PER_DUTY;

    // I term
    integ1 += KI * e1 * dt;
    integ2 += KI * e2 * dt;
    integ1 = clampf(integ1, -I_LIMIT, I_LIMIT);
    integ2 = clampf(integ2, -I_LIMIT, I_LIMIT);

    // duty output
    duty_out_1 = duty_ff_1 + KP * e1 + integ1;
    duty_out_2 = duty_ff_2 + KP * e2 + integ2;

    duty_out_1 = clampf(duty_out_1, 0.0f, (float)DUTY_MAX);
    duty_out_2 = clampf(duty_out_2, 0.0f, (float)DUTY_MAX);

    // slew-rate limit (chống giật)
    duty_out_1 = slew(duty_out_1, duty_prev_1);
    duty_out_2 = slew(duty_out_2, duty_prev_2);

    // apply to driver
    applyDuty(rpm_cmd_1, duty_out_1, PIN_ZF1, PIN_DR1, CH1, INVERT_DIR_1);
    applyDuty(rpm_cmd_2, duty_out_2, PIN_ZF2, PIN_DR2, CH2, INVERT_DIR_2);
  }

  // ---- publish ticks only ----
  if (now - last_pub_ms >= PUB_DT_MS) {
    last_pub_ms = now;
    Serial.print("e ");
    Serial.print((long)enc1.getCount());
    Serial.print(" ");
    Serial.println((long)enc2.getCount());
  }
}