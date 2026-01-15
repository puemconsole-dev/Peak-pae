#include <pop7.h>

// --- ตั้งค่าจำนวนเซนเซอร์ ---
#define NUM_SENSORS 5

// --- ตั้งค่าความเร็ว 3 ระดับ (ปรับตรงนี้!) ---
int speed_straight = 40;    // ระดับ 1: วิ่งทางตรง (A2 อยู่กลาง)
int speed_mild_turn = 25;   // ระดับ 2: เริ่มเอียง (A1 หรือ A3 เจอดำ)
int speed_sharp_turn = 50;  // ระดับ 3: หลุดโค้ง/หักศอก (A0 หรือ A4 เจอดำ)

// --- PID Control ---
float Kp = 0.05;  // ความไวการเลี้ยว
float Kd = 2;   // แรงหน่วงกันส่าย
float last_error = 0;

// --- ตัวแปรระบบ ---
int min_val[NUM_SENSORS];
int max_val[NUM_SENSORS];
int max_pwm = 100;

void setup() {
  // ขั้นตอนที่ 1: เตรียม Calibrate
  glcd(1, 0, "Press OK");
  glcd(2, 0, "Calibrate");
  sw_OK_press();
  
  beep();
  glcdClear();
  glcd(1, 0, "Auto Calib...");
  
  // ขั้นตอนที่ 2: Auto Calibration
  auto_calibrate();
  
  // จบการ Calibrate
  motor(1, 0); motor(2, 0);
  beep(); sleep(200); beep();
  
  // ขั้นตอนที่ 3: รอวิ่ง
  glcdClear();
  glcd(1, 0, "Calib OK!");
  glcd(2, 0, "Press Run");
  sw_OK_press();
  
  beep();
  glcdClear();
  glcd(1, 0, "Running...");
  sleep(500);
}

void loop() {
  // อ่านตำแหน่ง (0-4000, กลาง 2000)
  int position_val = read_position();
  int error = position_val - 2000;
  int abs_error = abs(error); // ดูแค่ค่าความห่าง ไม่สนซ้ายขวา
  
  // --- ระบบแยกความเร็ว 3 ระดับ (Zone Speed) ---
  int current_base_speed;

  if (abs_error >= 1500) {
    // โซน A0, A4 (ค่า Error สูงกว่า 1500) -> เจอโค้งโหด
    current_base_speed = speed_sharp_turn;
  } 
  else if (abs_error >= 500) {
    // โซน A1, A3 (ค่า Error ระหว่าง 500-1500) -> เจอโค้งธรรมดา
    current_base_speed = speed_mild_turn;
  } 
  else {
    // โซน A2 (ค่า Error น้อยกว่า 500) -> ทางตรง
    current_base_speed = speed_straight;
  }
  
  // --- คำนวณ PID ---
  int P = error;
  int D = error - last_error;
  int motor_diff = (Kp * P) + (Kd * D);
  
  last_error = error;
  
  // --- ผสมความเร็วพื้นฐาน + PID ---
  int speed_L = current_base_speed + motor_diff;
  int speed_R = current_base_speed - motor_diff;
  
  // ลิมิตมอเตอร์
  if (speed_L > max_pwm) speed_L = max_pwm;
  if (speed_L < -max_pwm) speed_L = -max_pwm;
  if (speed_R > max_pwm) speed_R = max_pwm;
  if (speed_R < -max_pwm) speed_R = -max_pwm;
  
  motor(1, speed_L);
  motor(2, speed_R);
}

// --- ฟังก์ชันเสริม (เหมือนเดิม) ---

void auto_calibrate() {
  for(int i=0; i<NUM_SENSORS; i++) {
    min_val[i] = 1023;
    max_val[i] = 0;
  }
  unsigned long start_time = millis();
  while(millis() - start_time < 5000) {
    int time_slot = (millis() - start_time) % 600;
    if (time_slot < 0) {
      motor(1, 35); motor(2, -35);
    } else {
      motor(1, 0); motor(2, 0);
    }
    for(int i=0; i<NUM_SENSORS; i++) {
      int val = analog(i); 
      if(val > max_val[i]) max_val[i] = val;
      if(val < min_val[i]) min_val[i] = val;
    }
  }
}

int read_position() {
  long weighted_sum = 0;
  long sum_val = 0;
  for(int i=0; i<NUM_SENSORS; i++) {
    int val = analog(i);
    long numerator = (long)(val - min_val[i]) * 1000;
    long denominator = (max_val[i] - min_val[i]);
    if (denominator == 0) denominator = 1; 
    int mapped_val = numerator / denominator;
    if(mapped_val < 0) mapped_val = 0;
    if(mapped_val > 1000) mapped_val = 1000;
    
    // mapped_val = 1000 - mapped_val; // เปิดบรรทัดนี้ถ้าพื้นดำ เส้นขาว

    weighted_sum += (long)mapped_val * (i * 1000);
    sum_val += mapped_val;
  }
  if (sum_val == 0) {
    if (last_error < 0) return 0; else return 4000;
  }
  return weighted_sum / sum_val;
}
