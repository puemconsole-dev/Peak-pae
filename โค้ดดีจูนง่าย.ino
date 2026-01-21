#include <pop7.h>

// =============================================================
//  ส่วนที่ 1: พื้นที่ปรับแต่ง
// =============================================================

int APPROACH_TIME = 700; 
int BACK_TIME_UTURN = 250; 

int NormalSpeed = 39;  
int SlowSpeed   = 21;  
int ADJUST_TIME = 0;   

//401 , 403 , 301 , 303 , 402 
//304 , 302 

// --- Mission List ---
int MissionList[] = {  
  402, 2 , 110, 2 , 100 , 1 , 401 , 403 , 301 , 303 , 1 , 120 , 3 , 110 , 5, 110 , 4 , 100 , 1 , 304 , 302 
  
};

int MissionLength = sizeof(MissionList) / sizeof(int);

int s1_Open = 55;    int s1_Close = 3;   
int s2_Open = 45;    int s2_Close = 108;  
int s3_Down = 0;     int s3_Up = 90;      

int BaseSpeed = NormalSpeed; 
int TurnSpeed = 30; 
float Kp = 1.5;  float Kd = 10.0;

// =============================================================
//  ส่วนที่ 2: ระบบภายใน
// =============================================================

// --- PID SENSORS (A0-A4) สำหรับเดินตามเส้น ---
#define NUM_SENSORS 5
int sensor_pin[] = {0, 1, 2, 3, 4}; 
int weight[]     = {-40, -20, 0, 20, 40};
int threshold[NUM_SENSORS];
int sensor_min[NUM_SENSORS], sensor_max[NUM_SENSORS];

// --- SIDE SENSORS (A5, A6) สำหรับเช็คแยก ---
int PIN_SIDE_L = 5; // เซนเซอร์แยกซ้าย
int PIN_SIDE_R = 6; // เซนเซอร์แยกขวา
int th_SideL = 500, th_SideR = 500; // ตัวแปรเก็บค่าแสงแยก
int min_SideL = 1023, max_SideL = 0;
int min_SideR = 1023, max_SideR = 0;

int last_error = 0;
int StepIndex = 0;   int CurrentCount = 0; 

#define CMD_TURN_RIGHT 100
#define CMD_TURN_LEFT  110
#define CMD_U_TURN     120 
#define CMD_STOP       200
#define CMD_GRIP       301 
#define CMD_RELEASE    302 
#define CMD_LIFT_UP    303 
#define CMD_LIFT_DOWN  304 
#define CMD_SET_SLOW   401 
#define CMD_SET_NORMAL 402 
#define CMD_TIMED_MOVE 403

// ฟังก์ชัน Servo
void smoothMoveGripper(int start1, int end1, int start2, int end2) {
  int steps = 40; 
  for (int i = 0; i <= steps; i++) {
    int c1 = map(i, 0, steps, start1, end1);
    int c2 = map(i, 0, steps, start2, end2);
    servo(1, c1); servo(2, c2); 
    sleep(20); 
  }
  servo(1, end1); servo(2, end2);
}
void smoothMoveLift(int startAngle, int endAngle) {
  int steps = 50; 
  for (int i = 0; i <= steps; i++) {
    int c = map(i, 0, steps, startAngle, endAngle);
    servo(3, c); sleep(15); 
  }
  servo(3, endAngle);
}

// ฟังก์ชันเดินรถ
int limit(int v, int min_val, int max_val) {
  if(v < min_val) return min_val; if(v > max_val) return max_val; return v;
}

void AutoCalibrate() {
  glcdClear(); setTextSize(1); glcd(0, 0, "Calibrating...");
  servo(1, s1_Open); servo(2, s2_Open); servo(3, s3_Down); 
  
  // รีเซ็ตค่า Min/Max ของ PID Sensors
  for(int i=0; i<NUM_SENSORS; i++) { sensor_min[i]=1023; sensor_max[i]=0; }
  
  // รีเซ็ตค่า Min/Max ของ Side Sensors (A5, A6)
  min_SideL = 1023; max_SideL = 0;
  min_SideR = 1023; max_SideR = 0;

  for(int t=0; t<200; t++) {
    if(t<60) motor(25,-25); else if(t<140) motor(-25,25); else motor(25,-25);
    
    // เก็บค่า PID Sensors (0-4)
    for(int i=0; i<NUM_SENSORS; i++) {
      int val = analog(sensor_pin[i]);
      if(val<sensor_min[i]) sensor_min[i]=val; if(val>sensor_max[i]) sensor_max[i]=val;
    }
    
    // เก็บค่า Side Sensors (5-6)
    int valL = analog(PIN_SIDE_L);
    int valR = analog(PIN_SIDE_R);
    if(valL < min_SideL) min_SideL = valL; if(valL > max_SideL) max_SideL = valL;
    if(valR < min_SideR) min_SideR = valR; if(valR > max_SideR) max_SideR = valR;

    sleep(10);
  }
  motor(0,0);
  
  // คำนวณ Threshold PID
  for(int i=0; i<NUM_SENSORS; i++) { threshold[i]=(sensor_min[i]+sensor_max[i])/2; }
  
  // คำนวณ Threshold Side Sensors
  th_SideL = (min_SideL + max_SideL) / 2;
  th_SideR = (min_SideR + max_SideR) / 2;

  glcdClear(); glcd(0,0,"Ready! Press OK"); OK();
}

int GetError() {
  long sum=0; int cnt=0;
  for(int i=0; i<NUM_SENSORS; i++) {
    if(analog(sensor_pin[i]) < threshold[i]) { sum+=weight[i]; cnt++; }
  }
  if(cnt>=4) return 0; if(cnt==0) return last_error;
  return sum/cnt;
}

void RunPID() {
  int error = GetError();
  int power = (int)(Kp*error + Kd*(error-last_error));
  int L = limit(BaseSpeed-power, -40, 40);
  int R = limit(BaseSpeed+power, -40, 40);
  motor(1,L); motor(2,R); last_error = error;
}

// *** ฟังก์ชันเลี้ยว ***
void ExecuteTurn(bool isRight) {
  motor(1,0); motor(2,0); sleep(200); beep();
  glcd(2,0, isRight ? "Turn R" : "Turn L");
   
  // เดินหน้าเล็กน้อยเพื่อให้ล้อหลังพ้นเส้น
  motor(1, BaseSpeed); motor(2, BaseSpeed); 
  sleep(20); 

  // หมุนตัว
  if(isRight) {
    sl(TurnSpeed); sleep(200); // หมุนออกมาก่อน
  } else {
    sr(TurnSpeed); sleep(200); 
  }

  // หมุนต่อจนกว่าจะเจอเส้นกลาง (ใช้ A2)
  unsigned long st = millis();
  if(isRight) {
    while(analog(2) > threshold[2] && millis() - st < 2000) sl(TurnSpeed); 
  } else {
    while(analog(2) > threshold[2] && millis() - st < 2000) sr(TurnSpeed);
  }
   
  motor(1, isRight ? TurnSpeed : -TurnSpeed); 
  motor(2, isRight ? -TurnSpeed : TurnSpeed); sleep(50); 
  ao(); sleep(300); last_error = 0;
}

void ExecuteUTurn() {
  motor(1,0); motor(2,0); beep();
  glcd(2,0, "Action: U-Turn");
  motor(1, -BaseSpeed); motor(2, -BaseSpeed);
  sleep(BACK_TIME_UTURN);
  sr(TurnSpeed); 
  sleep(550); 
  unsigned long st = millis();
  while(analog(2) > threshold[2] && millis() - st < 3000) { sr(TurnSpeed); }
  motor(1, -TurnSpeed); motor(2, TurnSpeed); sleep(50);
  ao(); sleep(300);
  last_error = 0;
}

// =============================================================
//  ส่วนที่ 3: MAIN LOOP
// =============================================================
void setup() { 
  setTextSize(2); 
  BaseSpeed = NormalSpeed; 
  AutoCalibrate(); 
}

void loop() {
  if (StepIndex >= MissionLength) { ao(); glcd(0,0,"Mission Err!"); while(1); }

  int Cmd = MissionList[StepIndex];

  // 1. คำสั่งพิเศษ
  if (Cmd == CMD_SET_SLOW)   { BaseSpeed = SlowSpeed;   beep(); StepIndex++; return; }
  if (Cmd == CMD_SET_NORMAL) { BaseSpeed = NormalSpeed; beep(); StepIndex++; return; }

  // 2. เดินจับเวลา
  if (Cmd == CMD_TIMED_MOVE) {
      unsigned long startTime = millis();
      while(millis() - startTime < APPROACH_TIME) {
        RunPID(); sleep(10);
      }
      motor(1,0); motor(2,0); sleep(320);
      StepIndex++; CurrentCount=0; return;
  }

  // 3. Servo
  if (Cmd >= 300) {
    motor(1,0); motor(2,0); 
    if(Cmd == CMD_GRIP) smoothMoveGripper(s1_Open, s1_Close, s2_Open, s2_Close);
    else if(Cmd == CMD_RELEASE) smoothMoveGripper(s1_Close, s1_Open, s2_Close, s2_Open);
    else if(Cmd == CMD_LIFT_UP) smoothMoveLift(s3_Down, s3_Up);
    else if(Cmd == CMD_LIFT_DOWN) smoothMoveLift(s3_Up, s3_Down);
    sleep(100); StepIndex++; return;
  }

  // 4. Turn / U-Turn (ทำงานทันทีเมื่อถึงบรรทัดนี้ใน Loop)
  if (Cmd == CMD_TURN_RIGHT) { ExecuteTurn(true);  StepIndex++; CurrentCount=0; return; }
  if (Cmd == CMD_TURN_LEFT)  { ExecuteTurn(false); StepIndex++; CurrentCount=0; return; }
  if (Cmd == CMD_U_TURN)     { ExecuteUTurn();     StepIndex++; CurrentCount=0; return; } 
  if (Cmd == CMD_STOP)       { ao(); glcdClear(); glcd(1,0,"FINISH"); beep(); while(1); }

  // 5. เดินตามเส้น + เช็คแยกด้วย A5/A6
  
  // อ่านค่าเซนเซอร์เสริม (A5, A6)
  bool SideL = analog(PIN_SIDE_L) < th_SideL;
  bool SideR = analog(PIN_SIDE_R) < th_SideR;
  
  // เจอแยก (ซ้าย A5 + ขวา A6 เจอเส้นดำพร้อมกัน)
  if(SideL && SideR) { 
      CurrentCount++; 
      beep(); 
      
      if(CurrentCount >= Cmd) {
         // --- ถึงเป้าหมายตามจำนวนนับ ---
         // เบรกและหยุด
         motor(1, -BaseSpeed); motor(2, -BaseSpeed); sleep(50);
         motor(1, 0); motor(2, 0); sleep(0);
         
         // ปรับระยะ
         if (ADJUST_TIME != 0) {
            if(ADJUST_TIME > 0) { motor(1,BaseSpeed); motor(2,BaseSpeed); sleep(ADJUST_TIME); }
            else { motor(1,-BaseSpeed); motor(2,-BaseSpeed); sleep(abs(ADJUST_TIME)); }
         }
         
         motor(1,0); motor(2,0); sleep(300); 
         StepIndex++; CurrentCount = 0;
      } else {
         // --- ยังไม่ถึง (ทางผ่าน) ---
         // เดินหน้าข้ามแยกไปเลย (Blind Forward) เพื่อไม่ให้นับซ้ำ
         // เพิ่มความแรงนิดนึงตอนข้ามแยก
         motor(1, BaseSpeed); motor(2, BaseSpeed); 
         sleep(150); // ดันให้พ้นเส้น A5/A6
      }
  } else {
      // ถ้าไม่เจอแยก ให้เดินตาม PID ปกติ (ใช้ A0-A4)
      RunPID();
  }
}
