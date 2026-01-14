#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

/* -------- Sensor pin -------- */
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define S4 A4

#define SENSOR_COUNT 5

/* -------- Variable -------- */
inline int sensorValue[SENSOR_COUNT];
inline int sensorMin[SENSOR_COUNT];
inline int sensorMax[SENSOR_COUNT];
inline int sensorThreshold[SENSOR_COUNT];
inline byte sensorDigital[SENSOR_COUNT];

/* -------- Function -------- */
inline void sensor_init() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
}

inline void sensor_read() {
  sensorValue[0] = analogRead(S0);
  sensorValue[1] = analogRead(S1);
  sensorValue[2] = analogRead(S2);
  sensorValue[3] = analogRead(S3);
  sensorValue[4] = analogRead(S4);
}

/* -------- Auto Calibration -------- */
inline void sensor_calibration_start() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
}

inline void sensor_calibration_update() {
  sensor_read();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValue[i] < sensorMin[i]) sensorMin[i] = sensorValue[i];
    if (sensorValue[i] > sensorMax[i]) sensorMax[i] = sensorValue[i];
  }
}

inline void sensor_calibration_end() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

/* -------- Convert -------- */
inline void sensor_to_digital() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorDigital[i] = (sensorValue[i] > sensorThreshold[i]) ? 1 : 0;
  }
}

#endif
