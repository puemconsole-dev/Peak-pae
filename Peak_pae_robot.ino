#include "motor.h"
#include "senser_read.h"

/* ---------- sensor reference (จำเป็นตาม motor.h) ---------- */
int ref   = 500;
int refA7 = 500;

void setup() {
  initial();        // ตั้งค่า pin motor
  sensor_init();    // ตั้งค่า pin sensor
}

void loop() {
  motor(1, 30);
  motor(2, 30);
}
