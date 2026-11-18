// coastdown_test.cpp
// Coast-down test for estimating rotor inertia J (and the friction ratios b/J, tau_c/J).
//
// Sequence for one test:
//   1. Wait until the motor is stationary (no encoder edge for STOP_TIMEOUT_US).
//   2. Drive the motor at a fixed PWM until its speed is steady (or until
//      <max_spinup_ms> has elapsed).
//   3. Set PWM = 0 (coast, no braking) and keep logging until the motor stops.
//
// Speed is measured with the "M/T" method: number of new encoder edges divided by
// the exact time between the first and last of those edges (timestamped in the ISR).
// This stays accurate at both high and low speed, unlike a fixed counting window.
//
// Serial protocol (115200 baud, newline-terminated):
//   PC -> Arduino:  "RUN <pwm 1-255> <max_spinup_ms>"   start one test
//                   "X"                                  abort / motor off
//   Arduino -> PC:  "#READY"                             after boot
//                   "#START,<pwm>,<max_spinup_ms>"       test accepted
//                   "#WAIT"                              motor still turning, waiting
//                                                        for it to stop (sent every 1 s)
//                   "D,<t_us>,<phase>,<count>,<edge_us>,<rpm>"   one sample
//                   "#CUT,<t_us>,<reason>"               PWM set to 0; reason is
//                                                        STEADY or MAX_TIME
//                   "#END,<reason>"                      STOPPED | TIMEOUT | ABORTED
//                   "#ERR,<message>"
// All times are microseconds since the motor was switched on.
// phase: 0 = spin-up (motor driven), 1 = coasting (PWM = 0)
// count: cumulative encoder edges; edge_us: time of the most recent edge.

#include <Arduino.h>

#include "pinout.h"

// ---- Configuration ----
#define ENCODER_PPR          50           // Pulses per revolution for the encoder
#define SERIAL_BAUD          115200
#define SAMPLE_PERIOD_US     10000UL      // 100 Hz logging
#define STOP_TIMEOUT_US      1000000UL    // no edge for 1 s => motor considered stopped
#define MAX_COAST_US         60000000UL   // hard limit on the coast phase (60 s)
#define MAX_WAIT_STOP_MS     120000UL     // give up if the motor won't stop before a run
#define MIN_SPINUP_MS        100UL
#define MAX_SPINUP_MS        60000UL
#define ENCODER_GLITCH_US    50UL         // ignore edges closer than this (electrical noise)

// Steady-state detection: the speed is averaged over consecutive blocks; the
// drive is cut when two consecutive block means differ by less than STEADY_TOL_PCT.
#define STEADY_BLOCK_SAMPLES 100          // 100 samples = 1 s per block
#define STEADY_TOL_PCT       0.3f

volatile uint32_t isr_count = 0;
volatile uint32_t isr_last_edge_us = 0;

void encoder_isr() {
  uint32_t now = micros();
  if (now - isr_last_edge_us < ENCODER_GLITCH_US) return;  // reject glitches
  isr_last_edge_us = now;
  isr_count++;
}

// Read both ISR variables atomically (32-bit reads are not atomic on AVR).
static void read_encoder(uint32_t &count, uint32_t &edge_us) {
  noInterrupts();
  count = isr_count;
  edge_us = isr_last_edge_us;
  interrupts();
}

static void motor_off() {
  analogWrite(MOTOR_PWM_PIN, 0);
}

static bool abort_requested() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'X' || c == 'x') return true;
  }
  return false;
}

// Block until no encoder edge has been seen for STOP_TIMEOUT_US.
// Returns false on abort or if the motor does not stop within MAX_WAIT_STOP_MS.
static bool wait_until_stopped() {
  const uint32_t start_ms = millis();
  uint32_t last_msg_ms = start_ms;
  while (true) {
    if (abort_requested()) {
      Serial.println(F("#END,ABORTED"));
      return false;
    }
    uint32_t count, edge_us;
    read_encoder(count, edge_us);
    if (micros() - edge_us >= STOP_TIMEOUT_US) return true;

    uint32_t now_ms = millis();
    if (now_ms - start_ms >= MAX_WAIT_STOP_MS) {
      Serial.println(F("#ERR,motor did not stop"));
      return false;
    }
    if (now_ms - last_msg_ms >= 1000UL) {
      Serial.println(F("#WAIT"));
      last_msg_ms = now_ms;
    }
  }
}

static void run_test(int pwm, uint32_t max_spinup_ms) {
  Serial.print(F("#START,"));
  Serial.print(pwm);
  Serial.print(',');
  Serial.println(max_spinup_ms);

  motor_off();
  if (!wait_until_stopped()) {
    motor_off();
    return;
  }

  uint32_t prev_count, prev_edge_us;
  read_encoder(prev_count, prev_edge_us);
  float rpm = 0.0f;

  // Steady-state detection
  float block_sum = 0.0f;
  uint16_t block_n = 0;
  float prev_block_mean = -1.0f;
  bool steady = false;

  const uint32_t max_spinup_us = max_spinup_ms * 1000UL;
  const uint32_t t0 = micros();
  uint32_t next_sample = t0;
  uint32_t cut_us = 0;
  uint8_t phase = 0;

  analogWrite(MOTOR_PWM_PIN, pwm);

  while (true) {
    if (abort_requested()) {
      motor_off();
      Serial.println(F("#END,ABORTED"));
      return;
    }

    uint32_t now = micros();
    if ((int32_t)(now - next_sample) < 0) continue;

    // Schedule the next sample; resync if serial output made us fall behind.
    if (now - next_sample > SAMPLE_PERIOD_US) next_sample = now + SAMPLE_PERIOD_US;
    else next_sample += SAMPLE_PERIOD_US;

    // Spin-up finished: cut the drive and start coasting.
    // cut_us uses the same timestamp as this sample so that (now - cut_us)
    // can never underflow.
    if (phase == 0 && (steady || now - t0 >= max_spinup_us)) {
      motor_off();
      cut_us = now;
      phase = 1;
      Serial.print(F("#CUT,"));
      Serial.print(cut_us - t0);
      Serial.println(steady ? F(",STEADY") : F(",MAX_TIME"));
    }

    // ---- Speed estimate (M/T method) ----
    uint32_t count, edge_us;
    read_encoder(count, edge_us);
    uint32_t dn = count - prev_count;

    if (dn > 0) {
      uint32_t dt = edge_us - prev_edge_us;
      if (dt > 0) {
        rpm = (float)dn * 60.0e6f / ((float)ENCODER_PPR * (float)dt);
      }
      prev_count = count;
      prev_edge_us = edge_us;
    } else {
      // No new edge: the true speed is at most one pulse over the time since the last edge.
      uint32_t since = micros() - prev_edge_us;
      if (since >= STOP_TIMEOUT_US) {
        rpm = 0.0f;
      } else if (since > 0) {
        float bound = 60.0e6f / ((float)ENCODER_PPR * (float)since);
        if (bound < rpm) rpm = bound;
      }
    }

    // ---- Steady-state detection (spin-up only) ----
    if (phase == 0) {
      block_sum += rpm;
      block_n++;
      if (block_n >= STEADY_BLOCK_SAMPLES) {
        float mean = block_sum / block_n;
        if (prev_block_mean > 0.0f && mean > 0.0f &&
            fabs(mean - prev_block_mean) / mean * 100.0f < STEADY_TOL_PCT) {
          steady = true;  // the cut happens at the next sample
        }
        prev_block_mean = mean;
        block_sum = 0.0f;
        block_n = 0;
      }
    }

    // ---- Log ----
    Serial.print(F("D,"));
    Serial.print(now - t0);
    Serial.print(',');
    Serial.print(phase);
    Serial.print(',');
    Serial.print(count);
    Serial.print(',');
    Serial.print((int32_t)(edge_us - t0));  // negative if the last edge predates the test
    Serial.print(',');
    Serial.println(rpm, 2);

    // ---- End conditions ----
    if (phase == 1) {
      if (rpm == 0.0f) {
        Serial.println(F("#END,STOPPED"));
        return;
      }
      if (now - cut_us >= MAX_COAST_US) {
        Serial.println(F("#END,TIMEOUT"));
        return;
      }
    }
  }
}

void setup() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  motor_off();
  pinMode(ENCONDER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCONDER_PIN), encoder_isr, RISING);

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("#READY"));
}

void loop() {
  if (Serial.available() == 0) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("X")) {
    motor_off();
    Serial.println(F("#OK,MOTOR_OFF"));
    return;
  }

  if (line.startsWith("RUN")) {
    int s1 = line.indexOf(' ');
    int s2 = (s1 >= 0) ? line.indexOf(' ', s1 + 1) : -1;
    if (s1 < 0 || s2 < 0) {
      Serial.println(F("#ERR,usage: RUN <pwm> <max_spinup_ms>"));
      return;
    }
    long pwm = line.substring(s1 + 1, s2).toInt();
    long max_spinup_ms = line.substring(s2 + 1).toInt();

    if (pwm < 1 || pwm > 255) {
      Serial.println(F("#ERR,pwm must be 1-255"));
      return;
    }
    if (max_spinup_ms < (long)MIN_SPINUP_MS || max_spinup_ms > (long)MAX_SPINUP_MS) {
      Serial.println(F("#ERR,max_spinup_ms out of range"));
      return;
    }
    run_test((int)pwm, (uint32_t)max_spinup_ms);
    return;
  }

  Serial.println(F("#ERR,unknown command"));
}
