#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/* ---------- Pin definition ---------- */
#define pwm_l 3
#define in1_l 4
#define in2_l 5

#define pwm_r 6
#define in1_r 7
#define in2_r 8

/* ---------- External reference ---------- */
extern int ref;
extern int refA7;

/* ---------- Variable ---------- */
inline int __motorLastpow = 0;

/* ---------- Function ---------- */
inline void initial() {
  pinMode(pwm_l, OUTPUT);
  pinMode(in1_l, OUTPUT);
  pinMode(in2_l, OUTPUT);

  pinMode(pwm_r, OUTPUT);
  pinMode(in1_r, OUTPUT);
  pinMode(in2_r, OUTPUT);
}

inline void motor(char ch, int pw) {
  pw = constrain(pw, -100, 100);
  __motorLastpow = pw;
  int p = map(abs(pw), 0, 100, 0, 255);

  if (ch == 1) {
    analogWrite(pwm_l, p);
    digitalWrite(in1_l, pw < 0);
    digitalWrite(in2_l, pw >= 0);
  }
  else if (ch == 2) {
    analogWrite(pwm_r, p);
    digitalWrite(in1_r, pw < 0);
    digitalWrite(in2_r, pw >= 0);
  }
}

inline void off_motor() {
  motor(1, 0);
  motor(2, 0);
}

/* ---------- basic move ---------- */
inline void run_go() {
  motor(1, 30);
  motor(2, 30);
}

inline void run_left() {
  motor(1, 20);
  motor(2, -20);
}

inline void run_right() {
  motor(1, -20);
  motor(2, 20);
}

inline void go(int p1, int p2, int t) {
  motor(1, p1);
  motor(2, p2);
  delay(t);
  off_motor();
}

inline void go1(int p1, int p2) {
  motor(1, p1);
  motor(2, p2);
}

#endif
