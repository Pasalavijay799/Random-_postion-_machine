// =============================================================
//  Teensy 4.1  —  Dual Motor STEP/DIR Slave  [v4.0]
//  Master: Raspberry Pi (USB-Serial)
//
//  Protocol:
//    Pi → Teensy : C,<rpm1>,<dir1>,<en1>,<rpm2>,<dir2>,<en2>\n
//    Pi → Teensy : STOP\n  |  RESET\n  |  STATUS\n
//    Teensy → Pi : E,<count1>,<count2>,<rpm1_actual>,<rpm2_actual>\n  (every 20ms)
//                  READY\n  (on boot)
//                  WATCHDOG\n  (on timeout)
//                  RESET_OK\n
//
//  FIXES IN v4.0:
//   1. BUG FIX: enc1_isr was incorrectly attached to M2_ENC_B — now enc2_isr
//      is correctly attached to both M2 encoder pins.
//   2. actual_rpm clamped to >= 0 on Teensy before sending — no negative values.
//   3. MICROSTEP updated to 32 (6400 steps/rev) for smoother low-speed motion.
//      Update TB6600 DIP switches: SW3=ON SW4=ON for 1/32 microstep.
//   4. RAMP_STEP_RPM reduced to 1.0 (was 2.0) to match finer microstep resolution.
//   5. All v3.0 stability fixes retained (int64_t enc, char[] buf, etc.)
// =============================================================

#include <Arduino.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define M1_STEP    2
#define M1_DIR     3
#define M1_ENA     4
#define M1_ENC_A  14
#define M1_ENC_B  15

#define M2_STEP    5
#define M2_DIR     6
#define M2_ENA     7
#define M2_ENC_A  17
#define M2_ENC_B  18

#define LED_PIN   13

// ─── CONSTANTS ──────────────────────────────────────────────
#define STEPS_PER_REV       200
#define MICROSTEP            32        // FIX #3: 1/32 microstep = 6400 steps/rev
                                       // TB6600 DIP: SW1=ON SW2=ON SW3=ON SW4=ON
#define FULL_STEPS_REV      (STEPS_PER_REV * MICROSTEP)   // 6400

#define MAX_RPM             100.0f
#define MIN_RPM               0.5f
#define WATCHDOG_MS         400UL
#define TELEMETRY_MS         20UL

#define RAMP_INTERVAL_US     5000UL
#define RAMP_STEP_RPM         1.0f     // FIX #4: halved to match finer microstep

#define REVERSAL_DWELL_MS    30UL
#define VEL_WINDOW_MS        50UL
#define ENC_CPR            2000        // Your encoder: 2000 counts/rev

// ─── QUADRATURE STATE MACHINE ────────────────────────────────
static const int8_t QEM[16] = {
      0,  -1,  +1,   0,
     +1,   0,   0,  -1,
     -1,   0,   0,  +1,
      0,  +1,  -1,   0
};

// ─── MOTOR STRUCT ────────────────────────────────────────────
struct Motor {
  uint8_t pin_step, pin_dir, pin_ena;
  uint8_t pin_enc_a, pin_enc_b;

  volatile int64_t enc_count;
  volatile uint8_t enc_state;

  int64_t  enc_snapshot;
  uint32_t vel_window_start_ms;
  float    actual_rpm;

  float   target_rpm;
  uint8_t target_dir;
  bool    enabled;

  float   current_rpm;
  uint8_t current_dir;

  enum class ReversalState : uint8_t {
    NONE, RAMPING_DOWN, DWELL, APPLY_DIR
  } reversal_state;
  uint8_t  reversal_target_dir;
  float    reversal_target_rpm;
  uint32_t reversal_dwell_start;

  elapsedMicros ramp_timer;
  IntervalTimer step_timer;
  bool          timer_running;
  float         timer_period_us;
};

Motor M1, M2;

// ─── ENCODER ISRs ─────────────────────────────────────────────
FASTRUN void enc1_isr() {
  uint8_t a    = digitalReadFast(M1_ENC_A);
  uint8_t b    = digitalReadFast(M1_ENC_B);
  uint8_t next = (a << 1) | b;
  M1.enc_count += QEM[(M1.enc_state << 2) | next];
  M1.enc_state  = next;
}

// FIX #1: Was enc1_isr attached to M2 pins — now correctly enc2_isr
FASTRUN void enc2_isr() {
  uint8_t a    = digitalReadFast(M2_ENC_A);
  uint8_t b    = digitalReadFast(M2_ENC_B);
  uint8_t next = (a << 1) | b;
  M2.enc_count += QEM[(M2.enc_state << 2) | next];
  M2.enc_state  = next;
}

// ─── STEP ISRs ──────────────────────────────────────────────
FASTRUN void step1_isr() { digitalToggleFast(M1_STEP); }
FASTRUN void step2_isr() { digitalToggleFast(M2_STEP); }

// ─── FREQUENCY CALCULATION ──────────────────────────────────
inline float rpm_to_half_period_us(float rpm) {
  if (rpm < MIN_RPM) return 0.0f;
  float sps = rpm * FULL_STEPS_REV / 60.0f;
  return 500000.0f / sps;
}

// ─── APPLY RPM TO TIMER ─────────────────────────────────────
void apply_rpm(Motor &m, void (*isr)()) {
  if (m.current_rpm < MIN_RPM || !m.enabled) {
    if (m.timer_running) {
      m.step_timer.end();
      m.timer_running   = false;
      m.timer_period_us = 0.0f;
      digitalWriteFast(m.pin_step, LOW);
    }
    return;
  }
  float hp = rpm_to_half_period_us(m.current_rpm);
  if (!m.timer_running) {
    if (m.step_timer.begin(isr, hp)) {
      m.timer_running   = true;
      m.timer_period_us = hp;
    }
  } else if (fabsf(hp - m.timer_period_us) > 0.3f) {
    m.step_timer.update(hp);
    m.timer_period_us = hp;
  }
}

// ─── MOTOR INIT ─────────────────────────────────────────────
void motor_init(Motor &m,
                uint8_t step, uint8_t dir, uint8_t ena,
                uint8_t encA, uint8_t encB) {
  m.pin_step  = step; m.pin_dir = dir; m.pin_ena = ena;
  m.pin_enc_a = encA; m.pin_enc_b = encB;

  m.enc_count           = 0LL;
  m.enc_snapshot        = 0LL;
  m.vel_window_start_ms = millis();
  m.actual_rpm          = 0.0f;
  m.enc_state           = 0;

  m.target_rpm  = 0.0f; m.target_dir  = 1; m.enabled     = false;
  m.current_rpm = 0.0f; m.current_dir = 1;

  m.reversal_state       = Motor::ReversalState::NONE;
  m.reversal_target_dir  = 1;
  m.reversal_target_rpm  = 0.0f;
  m.reversal_dwell_start = 0;

  m.timer_running   = false;
  m.timer_period_us = 0.0f;

  pinMode(step, OUTPUT); digitalWriteFast(step, LOW);
  pinMode(dir,  OUTPUT); digitalWriteFast(dir,  1);
  pinMode(ena,  OUTPUT); digitalWriteFast(ena,  HIGH); // HIGH = disabled on TB6600

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
}

void set_enabled(Motor &m, bool en) {
  m.enabled = en;
  digitalWriteFast(m.pin_ena, en ? LOW : HIGH);
}

// ─── REQUEST NEW TARGET ──────────────────────────────────────
void request_target(Motor &m, float rpm, uint8_t dir, bool en, void (*isr)()) {
  set_enabled(m, en);
  if (!en) {
    m.target_rpm     = 0.0f;
    m.reversal_state = Motor::ReversalState::NONE;
    return;
  }
  rpm = constrain(rpm, 0.0f, MAX_RPM);
  dir = dir ? 1 : 0;

  if (dir != m.current_dir && m.current_rpm > 0.1f) {
    if (m.reversal_state == Motor::ReversalState::NONE) {
      m.reversal_state      = Motor::ReversalState::RAMPING_DOWN;
      m.reversal_target_dir = dir;
      m.reversal_target_rpm = rpm;
      m.target_rpm          = 0.0f;
    } else {
      m.reversal_target_dir = dir;
      m.reversal_target_rpm = rpm;
    }
  } else if (m.reversal_state == Motor::ReversalState::NONE) {
    m.target_dir = dir;
    if (dir != m.current_dir) {
      digitalWriteFast(m.pin_dir, dir);
      m.current_dir = dir;
    }
    m.target_rpm = rpm;
  }
}

// ─── REVERSAL STATE MACHINE ──────────────────────────────────
void update_reversal(Motor &m, void (*isr)()) {
  if (m.reversal_state == Motor::ReversalState::NONE) return;
  switch (m.reversal_state) {
    case Motor::ReversalState::RAMPING_DOWN:
      if (m.current_rpm <= 0.05f) {
        m.reversal_dwell_start = millis();
        m.reversal_state = Motor::ReversalState::DWELL;
      }
      break;
    case Motor::ReversalState::DWELL:
      if ((millis() - m.reversal_dwell_start) >= REVERSAL_DWELL_MS)
        m.reversal_state = Motor::ReversalState::APPLY_DIR;
      break;
    case Motor::ReversalState::APPLY_DIR:
      digitalWriteFast(m.pin_dir, m.reversal_target_dir);
      m.current_dir = m.reversal_target_dir;
      m.target_dir  = m.reversal_target_dir;
      m.target_rpm  = m.reversal_target_rpm;
      m.reversal_state = Motor::ReversalState::NONE;
      break;
    default:
      m.reversal_state = Motor::ReversalState::NONE;
      break;
  }
}

// ─── RAMP ENGINE ─────────────────────────────────────────────
void update_ramp(Motor &m, void (*isr)()) {
  if (m.ramp_timer < RAMP_INTERVAL_US) return;
  m.ramp_timer = 0;

  float diff = m.target_rpm - m.current_rpm;
  if (fabsf(diff) < 0.1f) {
    m.current_rpm = m.target_rpm;
  } else {
    float step = RAMP_STEP_RPM;
    if (fabsf(diff) < RAMP_STEP_RPM * 3.0f) {
      step = fabsf(diff) * 0.35f;
      if (step < 0.1f) step = 0.1f;
    }
    m.current_rpm += (diff > 0.0f) ? step : -step;
    m.current_rpm  = constrain(m.current_rpm, 0.0f, MAX_RPM);
  }
  apply_rpm(m, isr);
}

// ─── VELOCITY FROM ENCODER ───────────────────────────────────
void update_actual_rpm(Motor &m) {
  uint32_t now     = millis();
  uint32_t elapsed = now - m.vel_window_start_ms;
  if (elapsed < VEL_WINDOW_MS) return;

  noInterrupts();
  int64_t current_count = m.enc_count;
  interrupts();

  int64_t delta         = current_count - m.enc_snapshot;
  m.enc_snapshot        = current_count;
  m.vel_window_start_ms = now;

  float rpm = (float)delta / (float)ENC_CPR * (60000.0f / (float)elapsed);
  // FIX #2: clamp to non-negative before sending
  m.actual_rpm = (rpm < 0.0f) ? -rpm : rpm;
}

// ─── GLOBAL TIMING ───────────────────────────────────────────
elapsedMillis watchdog_timer;
elapsedMillis telemetry_timer;
bool link_active    = false;
bool watchdog_fault = false;

elapsedMillis led_timer;
bool led_state = false;

// ─── SERIAL BUFFER (heap-free) ───────────────────────────────
#define CMD_BUF_SIZE 82
static char    cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

// ─── COMMAND PARSER ──────────────────────────────────────────
void handle_command(const char *line, uint8_t len) {
  if (len == 0) return;

  bool is_status = (len == 6 && memcmp(line, "STATUS", 6) == 0);
  if (!is_status) {
    watchdog_timer = 0;
    link_active    = true;
    watchdog_fault = false;
  }

  if (len == 4 && memcmp(line, "STOP", 4) == 0) {
    M1.target_rpm = 0.0f; M2.target_rpm = 0.0f;
    M1.reversal_state = Motor::ReversalState::NONE;
    M2.reversal_state = Motor::ReversalState::NONE;
    Serial.println("STOP_OK");
    return;
  }

  if (len == 5 && memcmp(line, "RESET", 5) == 0) {
    M1.step_timer.end(); M1.timer_running = false;
    M2.step_timer.end(); M2.timer_running = false;
    digitalWriteFast(M1_STEP, LOW); digitalWriteFast(M2_STEP, LOW);
    M1.target_rpm = 0.0f; M1.current_rpm = 0.0f;
    M2.target_rpm = 0.0f; M2.current_rpm = 0.0f;
    noInterrupts();
    M1.enc_count = 0LL; M1.enc_snapshot = 0LL;
    M2.enc_count = 0LL; M2.enc_snapshot = 0LL;
    interrupts();
    M1.reversal_state = Motor::ReversalState::NONE;
    M2.reversal_state = Motor::ReversalState::NONE;
    watchdog_fault = false;
    Serial.println("RESET_OK");
    return;
  }

  if (is_status) {
    noInterrupts();
    int64_t c1 = M1.enc_count;
    int64_t c2 = M2.enc_count;
    interrupts();
    Serial.print("S,");
    Serial.print(M1.current_rpm,1); Serial.print(",");
    Serial.print(M1.actual_rpm,1);  Serial.print(",");
    Serial.print(M1.current_dir);   Serial.print(",");
    Serial.print(M1.enabled?1:0);   Serial.print(",");
    Serial.print(M2.current_rpm,1); Serial.print(",");
    Serial.print(M2.actual_rpm,1);  Serial.print(",");
    Serial.print(M2.current_dir);   Serial.print(",");
    Serial.print(M2.enabled?1:0);   Serial.print(",");
    Serial.print((int32_t)c1);      Serial.print(",");
    Serial.println((int32_t)c2);
    return;
  }

  if (line[0] == 'C') {
    float rpm1, rpm2;
    int   dir1, en1, dir2, en2;
    if (sscanf(line, "C,%f,%d,%d,%f,%d,%d",
               &rpm1, &dir1, &en1, &rpm2, &dir2, &en2) != 6) {
      Serial.println("ERR,BAD_CMD");
      return;
    }
    request_target(M1, constrain(rpm1,0.f,MAX_RPM), dir1?1:0, en1!=0, step1_isr);
    request_target(M2, constrain(rpm2,0.f,MAX_RPM), dir2?1:0, en2!=0, step2_isr);
    return;
  }

  Serial.println("ERR,UNKNOWN");
}

// ─── LED ────────────────────────────────────────────────────
void update_led() {
  bool running = (M1.current_rpm > 0.5f && M1.enabled) ||
                 (M2.current_rpm > 0.5f && M2.enabled);
  uint32_t blink_ms = watchdog_fault ? 100 : (running ? 500 : 2100);
  uint32_t on_ms    = watchdog_fault ? 50  : (running ? 250 : 80);
  uint32_t t = led_timer % blink_ms;
  led_state = (t < on_ms);
  digitalWriteFast(LED_PIN, led_state ? HIGH : LOW);
}

// ─── SETUP ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, HIGH);

  motor_init(M1, M1_STEP, M1_DIR, M1_ENA, M1_ENC_A, M1_ENC_B);
  motor_init(M2, M2_STEP, M2_DIR, M2_ENA, M2_ENC_A, M2_ENC_B);

  // FIX #1: M2 encoder pins now correctly attach enc2_isr (not enc1_isr)
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), enc1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), enc1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), enc2_isr, CHANGE);   // was enc1_isr — BUG FIXED
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), enc2_isr, CHANGE);   // was enc1_isr — BUG FIXED

  memset(cmd_buf, 0, sizeof(cmd_buf));
  cmd_idx = 0;

  Serial.println("READY");
  digitalWriteFast(LED_PIN, LOW);
}

// ─── MAIN LOOP ──────────────────────────────────────────────
void loop() {

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buf[cmd_idx] = '\0';
        handle_command(cmd_buf, cmd_idx);
        cmd_idx = 0;
      }
    } else {
      if (cmd_idx < CMD_BUF_SIZE - 1) {
        cmd_buf[cmd_idx++] = c;
      } else {
        cmd_idx = 0;
        Serial.println("ERR,OVERFLOW");
      }
    }
  }

  if (link_active && watchdog_timer > WATCHDOG_MS) {
    M1.target_rpm = 0.0f; M2.target_rpm = 0.0f;
    M1.reversal_state = Motor::ReversalState::NONE;
    M2.reversal_state = Motor::ReversalState::NONE;
    link_active    = false;
    watchdog_fault = true;
    Serial.println("WATCHDOG");
  }

  update_reversal(M1, step1_isr);
  update_reversal(M2, step2_isr);
  update_ramp(M1, step1_isr);
  update_ramp(M2, step2_isr);

  if (telemetry_timer >= TELEMETRY_MS) {
    telemetry_timer = 0;
    update_actual_rpm(M1);
    update_actual_rpm(M2);

    noInterrupts();
    int64_t c1 = M1.enc_count;
    int64_t c2 = M2.enc_count;
    interrupts();

    Serial.print("E,");
    Serial.print((int32_t)c1);      Serial.print(",");
    Serial.print((int32_t)c2);      Serial.print(",");
    Serial.print(M1.actual_rpm, 1); Serial.print(",");
    Serial.println(M2.actual_rpm, 1);
  }

  update_led();
}
