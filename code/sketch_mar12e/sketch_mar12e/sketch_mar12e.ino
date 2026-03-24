/******************** 
 * Mega 2560 + 4 DC motors (2x L298N) + 4 quadrature encoders
 * Control source: ESP32-C3 -> Mega by UART
 * Commands: F / B / L / R / S
 ********************/

// ========================= 通信配置 =========================
// USB调试口
#define DEBUG_PORT Serial
// 和 ESP32-C3 通信口：Mega Serial2 = TX2 pin16, RX2 pin17
#define CMD_PORT   Serial2
const unsigned long CMD_BAUD = 115200;

// 超时自动停车（毫秒）
const unsigned long CMD_TIMEOUT_MS = 50000;

// ========================= 编码器/PI参数 =========================
const float PULSES_PER_REV = 790.0f;     // 继续用你当前实测值
const unsigned long SAMPLE_MS = 50;      // PI采样周期 50ms
float Kp = 0.50f;
float Ki = 0.82f;

const float integMin = -80.0f;
const float integMax =  80.0f;

float alpha = 0.93f;                     // 比原始更平滑一点
const int PWM_MIN = 74;                  // 比较温和，但还有起步力
const int PWM_MAX = 255;

// ========================= 运动目标参数 =========================
float RPM_FWD  = 30.0f;   // 比34低一点，减少顿挫
float RPM_TURN = 36.0f;   // 转向也略保守一点


// 如果以后你想做“缓转弯而不是原地转”，可以改成：
// 左转：左侧低速，右侧高速
// 这里先给你最直接最稳的：原地差速转向

// ========================= 电机引脚映射 =========================
struct Motor {
  uint8_t en, in1, in2;
  uint8_t encA, encB;
};

// 你当前版本里的映射
Motor M[4] = {
  {5, 22, 23,  2, 30},   // M1
  {6, 24, 25,  3, 31},   // M2
  {7, 26, 27, 18, 32},   // M3
  {8, 28, 29, 19, 33}    // M4
};

// ========================= 电机逻辑分组 =========================
// 默认假设：M1/M3 左侧，M2/M4 右侧
// 如果你车子装反了，就只改这里
const uint8_t LEFT_MOTORS[2]  = {0, 2};  // M1, M3
const uint8_t RIGHT_MOTORS[2] = {1, 3};  // M2, M4

// ========================= 软件方向修正 =========================
// motSign:
// +1 -> motorSetDir(true) 就是逻辑前进
// -1 -> 该电机的前后方向在软件里反过来
int8_t motSign[4] = { +1, +1, +1, +1 };

// encSign:
// +1 -> 编码器正方向与逻辑前进一致
// -1 -> 编码器符号反了，软件修正
// 根据你之前日志，先按 M1/M3 反向处理
int8_t encSign[4] = { -1, +1, -1, -1 };

// 每个电机的速度微调系数
// 后续如果发现某个轮子总快/总慢，就调这里
float trim[4] = {0.90f, 1.10f, 1.25f, 1.10f};

// ========================= 状态变量 =========================
volatile long encoderCount[4] = {0,0,0,0};

float rpm_set[4]  = {0,0,0,0};   // 每个电机目标RPM（可正可负）
float rpm_filt[4] = {0,0,0,0};
float integ[4]    = {0,0,0,0};
int   pwmOut[4]   = {0,0,0,0};

long lastCount[4] = {0,0,0,0};
unsigned long lastMs = 0;

char currentCmd = 'S';
unsigned long lastCmdMs = 0;

// ========================= 底层函数 =========================
inline void motorRawForward(uint8_t i){
  digitalWrite(M[i].in1, HIGH);
  digitalWrite(M[i].in2, LOW);
}

inline void motorRawBackward(uint8_t i){
  digitalWrite(M[i].in1, LOW);
  digitalWrite(M[i].in2, HIGH);
}

inline void motorSetDir(uint8_t i, bool logicalForward){
  bool realForward = logicalForward;
  if (motSign[i] < 0) realForward = !realForward;

  if (realForward) motorRawForward(i);
  else             motorRawBackward(i);
}

inline void motorStop(uint8_t i){
  analogWrite(M[i].en, 0);
}

inline void quadTick(uint8_t i){
  bool a = digitalRead(M[i].encA);
  bool b = digitalRead(M[i].encB);
  if (a == b) encoderCount[i]++;
  else        encoderCount[i]--;
}

void isrEnc0(){ quadTick(0); }
void isrEnc1(){ quadTick(1); }
void isrEnc2(){ quadTick(2); }
void isrEnc3(){ quadTick(3); }

// ========================= 高层运动指令 =========================
void setAllRPM(float m1, float m2, float m3, float m4){
  rpm_set[0] = m1 * trim[0];
  rpm_set[1] = m2 * trim[1];
  rpm_set[2] = m3 * trim[2];
  rpm_set[3] = m4 * trim[3];
}

void commandStop(){
  currentCmd = 'S';
  setAllRPM(0, 0, 0, 0);
}

void commandForward(){
  currentCmd = 'F';
  setAllRPM(+RPM_FWD, +RPM_FWD, +RPM_FWD, +RPM_FWD);
}

void commandBackward(){
  currentCmd = 'B';
  setAllRPM(-RPM_FWD, -RPM_FWD, -RPM_FWD, -RPM_FWD);
}

void commandLeft(){
  currentCmd = 'L';
  // 原地左转：左侧后退，右侧前进
  setAllRPM(-RPM_TURN, +RPM_TURN, -RPM_TURN, +RPM_TURN);
}

void commandRight(){
  currentCmd = 'R';
  // 原地右转：左侧前进，右侧后退
  setAllRPM(+RPM_TURN, -RPM_TURN, +RPM_TURN, -RPM_TURN);
}

void handleCommand(char c){
  switch (c){
    case 'F': commandForward();  break;
    case 'B': commandBackward(); break;
    case 'L': commandLeft();     break;
    case 'R': commandRight();    break;
    case 'S': commandStop();     break;
    default: return; // 忽略未知字符
  }

  lastCmdMs = millis();

  DEBUG_PORT.print("CMD = ");
  DEBUG_PORT.println(currentCmd);
}

// ========================= setup =========================
void setup() {
  DEBUG_PORT.begin(115200);
  CMD_PORT.begin(CMD_BAUD);

  DEBUG_PORT.println("Mega 2560 diff-drive + ESP32-C3 UART control");
  DEBUG_PORT.println("Commands: F B L R S");
  DEBUG_PORT.println("UART to ESP32-C3: Serial2 (TX2=16, RX2=17)");

  for(int i=0;i<4;i++){
    pinMode(M[i].en, OUTPUT);
    pinMode(M[i].in1, OUTPUT);
    pinMode(M[i].in2, OUTPUT);

    // 先用上拉，通常更稳
    pinMode(M[i].encA, INPUT_PULLUP);
    pinMode(M[i].encB, INPUT_PULLUP);

    motorSetDir(i, true);
    analogWrite(M[i].en, 0);

    pwmOut[i]   = 0;
    integ[i]    = 0;
    rpm_filt[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(M[0].encA), isrEnc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M[1].encA), isrEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M[2].encA), isrEnc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M[3].encA), isrEnc3, CHANGE);

  noInterrupts();
  for(int i=0;i<4;i++) encoderCount[i] = 0;
  interrupts();

  lastMs = millis();
  lastCmdMs = millis();
  commandStop();
}

// ========================= 读取ESP32命令 =========================
void readESP32Commands(){
  while (CMD_PORT.available() > 0){
    char c = CMD_PORT.read();

    // 兼容小写
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

    // 只接受这几个命令
    if (c=='F' || c=='B' || c=='L' || c=='R' || c=='S'){
      handleCommand(c);
    }
  }
}

// ========================= PI控制更新 =========================
void updateMotorControl(){
  unsigned long now = millis();
  if (now - lastMs < SAMPLE_MS) return;

  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  long countSnap[4];
  noInterrupts();
  for(int i=0;i<4;i++) countSnap[i] = encoderCount[i];
  interrupts();

  for(int i=0;i<4;i++){
    long dCount = countSnap[i] - lastCount[i];
    lastCount[i] = countSnap[i];

    // 编码器符号修正
    dCount *= encSign[i];

    float rpm_meas = 0.0f;
    if(dt > 0.0f){
      rpm_meas = ((float)dCount / PULSES_PER_REV) * (60.0f / dt);
    }

    // 滤波
    rpm_filt[i] = alpha * rpm_filt[i] + (1.0f - alpha) * rpm_meas;

    // 目标=0时，清积分并停止
    if (fabs(rpm_set[i]) < 0.5f){
      integ[i] = 0;
      pwmOut[i] = 0;
      analogWrite(M[i].en, 0);
      continue;
    }

    // 根据目标正负决定逻辑方向
    bool logicalForward = (rpm_set[i] >= 0.0f);
    motorSetDir(i, logicalForward);

    // 用“绝对值速度”做 PI，方向单独控制
    float target = fabs(rpm_set[i]);
    float meas   = fabs(rpm_filt[i]);
    float err = target - meas;

    integ[i] += err * dt;
    if(integ[i] > integMax) integ[i] = integMax;
    if(integ[i] < integMin) integ[i] = integMin;

    float u = Kp * err + Ki * integ[i];
    int pwm = (int)u;

    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < 0)       pwm = 0;

    if(target > 1.0f && pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;

    pwmOut[i] = pwm;
    analogWrite(M[i].en, pwmOut[i]);
  }

  // 调试输出
  DEBUG_PORT.print("CMD=");
  DEBUG_PORT.print(currentCmd);
  DEBUG_PORT.print(" | ");

  for(int i=0;i<4;i++){
    DEBUG_PORT.print("M"); DEBUG_PORT.print(i+1);
    DEBUG_PORT.print(" set="); DEBUG_PORT.print(rpm_set[i],1);
    DEBUG_PORT.print(" f=");   DEBUG_PORT.print(rpm_filt[i],1);
    DEBUG_PORT.print(" pwm="); DEBUG_PORT.print(pwmOut[i]);
    DEBUG_PORT.print(" | ");
  }
  DEBUG_PORT.println();
}

// ========================= loop =========================
void loop() {
  readESP32Commands();
  updateMotorControl();
}
