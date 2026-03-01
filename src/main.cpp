#include <Arduino.h>

static const int PIN_EN  = 19;
static const int PIN_ZF  = 18;
static const int PIN_DR  = 5;

static const int PIN_SP1 = 32;
static const int PIN_SP2 = 33;

// ===== Encoder config =====
// PPR = số xung (edge) trên kênh bạn đang đếm cho mỗi vòng.
// Nếu bạn interrupt trên SP1 RISING thì thường là 1x decode (chỉ 1 cạnh/kênh).
// Hãy chỉnh đúng theo datasheet/driver.
static const float ENCODER_PPR = 1000.0f;   // <-- sửa số này cho đúng

volatile long ticks = 0;

void IRAM_ATTR isr_sp1_rise() {
  // Quadrature direction
  if (digitalRead(PIN_SP2)) ticks--;
  else                      ticks++;
}

// ===== LEDC for DR pulse =====
static const int PWM_CH  = 0;
static const int PWM_RES = 8;

void setDRFreq(uint32_t f_hz) {
  ledcSetup(PWM_CH, f_hz, PWM_RES);
  ledcAttachPin(PIN_DR, PWM_CH);

  // duty 8-bit: 0..255
  // 50% thì ~128. Bạn đang dùng 20 => ~7.8%
  ledcWrite(PWM_CH, 50);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_ZF, OUTPUT);

  // enable/dir (tuỳ driver active-low hay active-high)
  digitalWrite(PIN_EN, HIGH);
  digitalWrite(PIN_ZF, HIGH);

  pinMode(PIN_SP1, INPUT_PULLUP);
  pinMode(PIN_SP2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SP1), isr_sp1_rise, RISING);

  // PWM chạy luôn khi start (đỡ phải chờ 2s)
  setDRFreq(100);

  Serial.println("Start: EN set, ZF set, DR running, reading encoder -> RPM");
}

void loop() {
  // ===== sweep DR frequency mỗi 2s =====
  static uint32_t f = 100;
  static uint32_t lastChange = 0;

  if (millis() - lastChange > 2000) {
    lastChange = millis();
    setDRFreq(f);
    Serial.printf("[CMD] DR freq = %lu Hz\n", (unsigned long)f);

    f += 200;
    if (f > 2000) f = 100;
  }

  // ===== RPM estimation =====
  static long lastTicks = 0;
  static uint32_t lastMs = 0;

  const uint32_t Ts_ms = 200;  // thời gian lấy mẫu RPM

  uint32_t now = millis();
  if (now - lastMs >= Ts_ms) {
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    long t;
    noInterrupts(); t = ticks; interrupts();

    long d = t - lastTicks;
    lastTicks = t;

    float tick_per_s = d / dt;

    // revolutions per second = ticks/s / PPR
    float rps = tick_per_s / ENCODER_PPR;

    // RPM
    float rpm = rps * 60.0f;

    Serial.printf("[FB] ticks=%ld dTicks=%ld tick/s=%.1f RPM=%.2f\n",
                  t, d, tick_per_s, rpm);
  }
}