#include "comm.h"
#include "motor.h"

// ===================== 速度参数 =====================
// 默认档：已经很猛
int speedStraight = 252;
int speedMiddle   = 228;
int speedCorner   = 198;
int speedRecover  = 170;
int maxPWM        = 255;

// ===================== 控制参数 =====================
// 更激进，减少保守修正
float Kp = 35.5;
float Kd = 13.0;

// 误差滤波：比之前更小，更灵敏
float filteredError = 0.0;
float lastFilteredError = 0.0;

// 丢线恢复记忆
float lastGoodError = 0.0;
unsigned long lastSeenLineMs = 0;
int lastTurnDir = 0;   // -1 左, 0 中, 1 右

bool running = false;

// 左右轮补偿
int leftBias  = 0;
int rightBias = 10;

// ===================== 读一次原始7路 =====================
void readSensorsOnce(int s[7])
{
    reload_shift_reg();
    s[0] = sensor.ir_left_3;
    s[1] = sensor.ir_left_2;
    s[2] = sensor.ir_left_1;
    s[3] = sensor.ir_mid;
    s[4] = sensor.ir_right_1;
    s[5] = sensor.ir_right_2;
    s[6] = sensor.ir_right_3;
}

// ===================== 多次采样多数表决 =====================
// 赛道不平整，所以仍然保留3次，但后面的控制更激进
void readSensorsFiltered(int s[7])
{
    int sum[7] = {0,0,0,0,0,0,0};
    int t[7];

    readSensorsOnce(t);
    for (int i = 0; i < 7; i++) sum[i] += t[i];

    readSensorsOnce(t);
    for (int i = 0; i < 7; i++) sum[i] += t[i];

    readSensorsOnce(t);
    for (int i = 0; i < 7; i++) sum[i] += t[i];

    for (int i = 0; i < 7; i++)
        s[i] = (sum[i] >= 2) ? 1 : 0;
}

bool hasLine(int s[7])
{
    for (int i = 0; i < 7; i++)
        if (s[i]) return true;
    return false;
}

int activeCount(int s[7])
{
    int c = 0;
    for (int i = 0; i < 7; i++)
        if (s[i]) c++;
    return c;
}

float calcWeightedError(int s[7], int &count)
{
    // 再强化一点外侧权重
    int w[7] = {-7, -4, -1, 0, 1, 4, 7};
    long sum = 0;
    count = 0;

    for (int i = 0; i < 7; i++)
    {
        if (s[i])
        {
            sum += w[i];
            count++;
        }
    }

    if (count == 0) return 0.0f;
    return (float)sum / count;
}

void driveLR(int left, int right)
{
    left  += leftBias;
    right += rightBias;

    if (left > maxPWM) left = maxPWM;
    if (left < -maxPWM) left = -maxPWM;
    if (right > maxPWM) right = maxPWM;
    if (right < -maxPWM) right = -maxPWM;

    motor_set_PWM(left, right);
}

// ===================== 连续丢线找回 =====================
void recoverContinuousFast()
{
    unsigned long dt = millis() - lastSeenLineMs;

    if (dt < 25)
    {
        if (lastGoodError < -0.20f || lastTurnDir < 0)
            driveLR(115, speedRecover);
        else if (lastGoodError > 0.20f || lastTurnDir > 0)
            driveLR(speedRecover, 115);
        else
            driveLR(155, 155);
        return;
    }

    if (dt < 80)
    {
        if (lastGoodError < 0 || lastTurnDir < 0)
            driveLR(65, speedRecover + 28);
        else
            driveLR(speedRecover + 28, 65);
        return;
    }

    if (lastGoodError < 0 || lastTurnDir < 0)
        driveLR(-80, 170);
    else
        driveLR(170, -80);
}

// ===================== 主循迹 =====================
void followTrackMaxSpeed()
{
    int s[7];
    readSensorsFiltered(s);

    bool onLine = hasLine(s);
    int cnt = activeCount(s);

    if (!onLine)
    {
        recoverContinuousFast();
        return;
    }

    lastSeenLineMs = millis();

    float err = calcWeightedError(s, cnt);
    lastGoodError = err;

    if (err < -0.10f) lastTurnDir = -1;
    else if (err > 0.10f) lastTurnDir = 1;
    else lastTurnDir = 0;

    // 更灵敏
    filteredError = 0.42f * filteredError + 0.58f * err;

    // 死区再缩小
    if (filteredError > -0.03f && filteredError < 0.03f)
        filteredError = 0.0f;

    float dErr = filteredError - lastFilteredError;
    float turn = Kp * filteredError + Kd * dErr;
    lastFilteredError = filteredError;

    // 更高的转向上限
    if (turn > 135) turn = 135;
    if (turn < -135) turn = -135;

    int base;

    // 真直线
    if (cnt == 1 && s[3])
    {
        base = speedStraight;
    }
    // 中间区域，仍然尽量快
    else if ((s[2] && s[3]) || (s[3] && s[4]) || (cnt >= 2 && s[3]))
    {
        base = speedMiddle;
    }
    // 弯道
    else
    {
        base = speedCorner;
    }

    // 同时亮很多时，少减速
    if (cnt >= 4) base -= 6;
    else if (cnt == 3) base -= 3;

    // 外侧触发时直接更激进修正
    if (s[0] && !s[6]) turn -= 34;
    if (s[6] && !s[0]) turn += 34;

    int leftPWM  = base + (int)turn;
    int rightPWM = base - (int)turn;

    driveLR(leftPWM, rightPWM);
}

void setup()
{
    shift_reg_init();
    motor_init();
}

void loop()
{
    reload_shift_reg();

    // KEY1：开始/停止
    if (sensor.key_1)
    {
        while (sensor.key_1) reload_shift_reg();
        delay(8);
        running = !running;
    }

    // KEY2：真正的最猛档
    if (sensor.key_2)
    {
        while (sensor.key_2) reload_shift_reg();
        delay(8);

        if (speedStraight == 252)
        {
            speedStraight = 255;
            speedMiddle   = 238;
            speedCorner   = 206;
            speedRecover  = 178;
        }
        else
        {
            speedStraight = 252;
            speedMiddle   = 228;
            speedCorner   = 198;
            speedRecover  = 170;
        }
    }

    if (running)
        followTrackMaxSpeed();
    else
        driveLR(0, 0);
}
