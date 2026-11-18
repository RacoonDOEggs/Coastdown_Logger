// coastdown_test.cpp
// Coast-down test for estimating rotor inertia J (and the friction ratios b/J, tau_c/J).
//
// Sequence for one test:
//   1. Drive the motor at a fixed PWM for <spinup_ms> so it reaches steady speed.
//   2. Set PWM = 0 (coast, no braking) and keep logging until the motor stops.
//
// Speed is measured with the "M/T" method: number of new encoder edges divided by
// the exact time between the first and last of those edges (timestamped in the ISR).
// This stays accurate at both high and low speed, unlike a fixed counting window.
//
// Serial protocol (115200 baud, newline-terminated):
//   PC -> Arduino:  "RUN <pwm 1-255> <spinup_ms>"   start one test
//                   "X"                              abort / motor off
//   Arduino -> PC:  "#READY"                         after boot
//                   "#START,<pwm>,<spinup_ms>"       test accepted
//                   "D,<t_us>,<phase>,<count>,<edge_us>,<rpm>"   one sample
//                   "#CUT,<t_us>"                    instant PWM was set to 0
//                   "#END,<reason>"                  STOPPED | TIMEOUT | ABORTED
//                   "#ERR,<message>"
// All times are microseconds since the start of the test.
// phase: 0 = spin-up (motor driven), 1 = coasting (PWM = 0)
// count: cumulative encoder edges; edge_us: time of the most recent edge.

#include <Arduino.h>

#include "pinout.h"

// ---- Configuration ----
#define ENCODER_PPR        50           // Pulses per revolution for the encoder
#define SERIAL_BAUD        115200
#define SAMPLE_PERIOD_US   10000UL      // 100 Hz logging
#define STOP_TIMEOUT_US    1000000UL    // no edge for 1 s => motor considered stopped
#define MAX_COAST_US       60000000UL   // hard limit on the coast phase (60 s)
#define MIN_SPINUP_MS      100UL
#define MAX_SPINUP_MS      60000UL
#define ENCODER_GLITCH_US  50UL         // ignore edges closer than this (electrical noise)

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

static void run_test(int pwm, uint32_t spinup_ms) {
  Serial.print(F("#START,"));
  Serial.print(pwm);
  Serial.print(',');
  Serial.println(spinup_ms);

  uint32_t prev_count, prev_edge_us;
  read_encoder(prev_count, prev_edge_us);
  float rpm = 0.0f;

  const uint32_t t0 = micros();
  const uint32_t spinup_us = spinup_ms * 1000UL;
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
    if (phase == 0 && now - t0 >= spinup_us) {
      motor_off();
      cut_us = micros();
      phase = 1;
      Serial.print(F("#CUT,"));
      Serial.println(cut_us - t0);
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
      Serial.println(F("#ERR,usage: RUN <pwm> <spinup_ms>"));
      return;
    }
    long pwm = line.substring(s1 + 1, s2).toInt();
    long spinup_ms = line.substring(s2 + 1).toInt();

    if (pwm < 1 || pwm > 255) {
      Serial.println(F("#ERR,pwm must be 1-255"));
      return;
    }
    if (spinup_ms < (long)MIN_SPINUP_MS || spinup_ms > (long)MAX_SPINUP_MS) {
      Serial.println(F("#ERR,spinup_ms out of range"));
      return;
    }
    run_test((int)pwm, (uint32_t)spinup_ms);
    return;
  }

  Serial.println(F("#ERR,unknown command"));
}
