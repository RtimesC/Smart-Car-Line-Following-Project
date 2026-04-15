#include <Arduino.h>
#include "comm.h"
#include "motor.h"

/*
  7路红外巡线程序（适配你现有的 comm.* / motor.*）

  传感器顺序：
  ir_left_3, ir_left_2, ir_left_1, ir_mid, ir_right_1, ir_right_2, ir_right_3

  使用方法：
  1. 把本文件命名为 line_following_track.ino
  2. 与 comm.cpp/.h、motor.cpp/.h 放在同一个 Arduino 工程目录
  3. 编译下载
  4. 上电后默认静止，按一次 key_1 启动，再按一次 key_1 暂停

  如果小车完全“反着巡线”或者一上来就偏离：
  - 先改 LINE_ACTIVE_LEVEL（HIGH <-> LOW）
  - 再看电机方向是否相反，必要时交换 left/right 的符号
*/

// ==========================
// 可调参数
// ==========================
const uint8_t LINE_ACTIVE_LEVEL = HIGH;   // 黑线被检测到时的电平；若相反就改成 LOW

const int BASE_SPEED_STRAIGHT = 145;      // 直线基础速度
const int BASE_SPEED_CURVE    = 125;      // 普通弯道速度
const int BASE_SPEED_SHARP    = 105;      // 急弯 / 发卡弯速度

const int MAX_PWM = 220;
const int MIN_PWM = -220;

const float KP = 30.0f;
const float KD = 72.0f;
const float KI = 0.0f;

const unsigned long LOOP_DT_MS           = 5;    // 主循环周期
const unsigned long LOST_PIVOT_DELAY_MS  = 140;  // 丢线一小段时间后开始原地找线
const int SEARCH_FORWARD_PWM             = 125;  // 轻度找线时的前进速度
const int SEARCH_PIVOT_FAST              = 150;  // 原地找线快轮
const int SEARCH_PIVOT_SLOW              = -115; // 原地找线慢/反转轮

const bool ENABLE_BUMPER_STOP = true;            // 碰撞开关触发后停车
const bool ENABLE_SERIAL_DEBUG = false;          // 调试传感器时改成 true

// 权重：从左到右
const int weights[7] = {-3, -2, -1, 0, 1, 2, 3};

// ==========================
// 控制状态
// ==========================
float lastError = 0.0f;
float lastValidError = 0.0f;
float integral = 0.0f;
unsigned long lastSeenLineMs = 0;
unsigned long lastLoopMs = 0;
bool carEnabled = false;
bool lastKey1 = false;

// ==========================
// 基础函数
// ==========================
int clampPWM(int v)
{
  if (v > MAX_PWM) return MAX_PWM;
  if (v < MIN_PWM) return MIN_PWM;
  return v;
}

bool lineDetected(uint8_t raw)
{
  return raw == LINE_ACTIVE_LEVEL;
}

void resetController()
{
  lastError = 0.0f;
  lastValidError = 0.0f;
  integral = 0.0f;
  lastSeenLineMs = millis();
}

void stopCar()
{
  motor_set_PWM(0, 0);
}

void setMotor(int leftPWM, int rightPWM)
{
  motor_set_PWM(clampPWM(leftPWM), clampPWM(rightPWM));
}

bool bumperTriggered()
{
  return sensor.switcher_front_left_1 || sensor.switcher_front_left_2 ||
         sensor.switcher_front_right_1 || sensor.switcher_front_right_2;
}

void fillLineArray(bool s[7])
{
  s[0] = lineDetected(sensor.ir_left_3);
  s[1] = lineDetected(sensor.ir_left_2);
  s[2] = lineDetected(sensor.ir_left_1);
  s[3] = lineDetected(sensor.ir_mid);
  s[4] = lineDetected(sensor.ir_right_1);
  s[5] = lineDetected(sensor.ir_right_2);
  s[6] = lineDetected(sensor.ir_right_3);
}

int countActive(const bool s[7])
{
  int cnt = 0;
  for (int i = 0; i < 7; ++i)
  {
    if (s[i]) cnt++;
  }
  return cnt;
}

float computeError(const bool s[7], int activeCount)
{
  if (activeCount <= 0) return lastValidError;

  int sum = 0;
  for (int i = 0; i < 7; ++i)
  {
    if (s[i]) sum += weights[i];
  }
  return (float)sum / (float)activeCount;
}

bool isSharpLeft(const bool s[7])
{
  // 左侧极端点亮，右侧基本没看到线，按急左弯处理
  return (s[0] && s[1] && !s[4] && !s[5] && !s[6]) ||
         (s[0] && !s[3] && !s[4] && !s[5] && !s[6]) ||
         (s[0] && s[1] && s[2] && !s[5] && !s[6]);
}

bool isSharpRight(const bool s[7])
{
  // 右侧极端点亮，左侧基本没看到线，按急右弯处理
  return (s[6] && s[5] && !s[2] && !s[1] && !s[0]) ||
         (s[6] && !s[3] && !s[2] && !s[1] && !s[0]) ||
         (s[6] && s[5] && s[4] && !s[1] && !s[0]);
}

void searchLine()
{
  int dir = (lastValidError >= 0.0f) ? 1 : -1; // +1: 线在右边，-1: 线在左边
  unsigned long lostMs = millis() - lastSeenLineMs;

  if (lostMs < LOST_PIVOT_DELAY_MS)
  {
    // 先带一点前进的小幅修正，避免刚出弯就原地打转
    int leftPWM  = SEARCH_FORWARD_PWM + dir * 35;
    int rightPWM = SEARCH_FORWARD_PWM - dir * 35;
    setMotor(leftPWM, rightPWM);
  }
  else
  {
    // 彻底丢线后按上次看到的方向原地找线
    if (dir > 0)
    {
      // 右找线
      setMotor(SEARCH_PIVOT_FAST, SEARCH_PIVOT_SLOW);
    }
    else
    {
      // 左找线
      setMotor(SEARCH_PIVOT_SLOW, SEARCH_PIVOT_FAST);
    }
  }
}

void followLineStep()
{
  bool s[7];
  fillLineArray(s);

  int activeCount = countActive(s);

  if (activeCount == 0)
  {
    searchLine();
    return;
  }

  float error = computeError(s, activeCount);
  float absError = fabs(error);

  bool sharpLeft = isSharpLeft(s);
  bool sharpRight = isSharpRight(s);

  int baseSpeed = BASE_SPEED_STRAIGHT;
  if (sharpLeft || sharpRight || absError >= 2.3f)
  {
    baseSpeed = BASE_SPEED_SHARP;
  }
  else if (absError >= 1.1f)
  {
    baseSpeed = BASE_SPEED_CURVE;
  }

  // PID
  integral += error;
  if (integral > 8.0f) integral = 8.0f;
  if (integral < -8.0f) integral = -8.0f;

  float derivative = error - lastError;
  float correction = KP * error + KD * derivative + KI * integral;

  // 对赛道里的急弯再额外加一点转向量，避免切弯过大
  if (sharpLeft)  correction -= 25.0f;
  if (sharpRight) correction += 25.0f;

  int leftPWM  = (int)(baseSpeed + correction);
  int rightPWM = (int)(baseSpeed - correction);

  setMotor(leftPWM, rightPWM);

  lastError = error;
  lastValidError = error;
  lastSeenLineMs = millis();

  if (ENABLE_SERIAL_DEBUG)
  {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 60)
    {
      lastPrint = millis();
      Serial.print("S: ");
      for (int i = 0; i < 7; ++i)
      {
        Serial.print(s[i]);
        Serial.print(' ');
      }
      Serial.print(" | err=");
      Serial.print(error, 2);
      Serial.print(" | L=");
      Serial.print(leftPWM);
      Serial.print(" R=");
      Serial.println(rightPWM);
    }
  }
}

void handleKeys()
{
  bool nowKey1 = sensor.key_1;

  if (nowKey1 && !lastKey1)
  {
    carEnabled = !carEnabled;
    if (!carEnabled)
    {
      stopCar();
    }
    resetController();
    delay(180); // 简单消抖
  }

  lastKey1 = nowKey1;
}

// ==========================
// Arduino
// ==========================
void setup()
{
  if (ENABLE_SERIAL_DEBUG)
  {
    Serial.begin(115200);
  }

  shift_reg_init();
  motor_init();

  stopCar();
  delay(300);
  reload_shift_reg();
  resetController();
}

void loop()
{
  if (millis() - lastLoopMs < LOOP_DT_MS)
  {
    return;
  }
  lastLoopMs = millis();

  reload_shift_reg();
  handleKeys();

  if (!carEnabled)
  {
    stopCar();
    return;
  }

  if (ENABLE_BUMPER_STOP && bumperTriggered())
  {
    stopCar();
    carEnabled = false;
    return;
  }

  followLineStep();
}
