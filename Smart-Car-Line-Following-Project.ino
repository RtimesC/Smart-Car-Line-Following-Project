#include "comm.h"
#include "motor.h"

int speedStraight = 255;
int speedMiddle   = 242;
int speedCorner   = 212;
int speedRecover  = 182;
int maxPWM        = 255;

float Kp = 37.5;
float Kd = 11.0;

float filteredError = 0.0;
float lastFilteredError = 0.0;

float lastGoodError = 0.0;
unsigned long lastSeenLineMs = 0;
int lastTurnDir = 0;

bool running = false;

int leftBias  = 0;
int rightBias = 10;

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

void readSensorsFilteredFast(int s[7])
{
    int a[7], b[7];
    readSensorsOnce(a);
    readSensorsOnce(b);

    for (int i = 0; i < 7; i++)
    {
        if (a[i] == b[i]) s[i] = a[i];
        else s[i] = a[i];
    }
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
    int w[7] = {-8, -4, -1, 0, 1, 4, 8};
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

void recoverContinuousFast()
{
    unsigned long dt = millis() - lastSeenLineMs;

    if (dt < 20)
    {
        if (lastGoodError < -0.18f || lastTurnDir < 0)
            driveLR(125, speedRecover);
        else if (lastGoodError > 0.18f || lastTurnDir > 0)
            driveLR(speedRecover, 125);
        else
            driveLR(165, 165);
        return;
    }

    if (dt < 70)
    {
        if (lastGoodError < 0 || lastTurnDir < 0)
            driveLR(75, speedRecover + 30);
        else
            driveLR(speedRecover + 30, 75);
        return;
    }

    if (lastGoodError < 0 || lastTurnDir < 0)
        driveLR(-85, 175);
    else
        driveLR(175, -85);
}

void followTrackMaxPlus()
{
    int s[7];
    readSensorsFilteredFast(s);

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

    if (err < -0.08f) lastTurnDir = -1;
    else if (err > 0.08f) lastTurnDir = 1;
    else lastTurnDir = 0;

    filteredError = 0.35f * filteredError + 0.65f * err;

    if (filteredError > -0.02f && filteredError < 0.02f)
        filteredError = 0.0f;

    float dErr = filteredError - lastFilteredError;
    float turn = Kp * filteredError + Kd * dErr;
    lastFilteredError = filteredError;

    if (turn > 155) turn = 155;
    if (turn < -155) turn = -155;

    int base;

    if (cnt == 1 && s[3])
    {
        base = speedStraight;
    }
    else if ((s[2] && s[3]) || (s[3] && s[4]) || (cnt >= 2 && s[3]))
    {
        base = speedMiddle;
    }
    else
    {
        base = speedCorner;
    }

    if (cnt >= 4) base -= 4;
    else if (cnt == 3) base -= 2;

    if (s[0] && !s[6]) turn -= 44;
    if (s[6] && !s[0]) turn += 44;

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

    if (sensor.key_1)
    {
        while (sensor.key_1) reload_shift_reg();
        delay(6);
        running = !running;
    }

    if (sensor.key_2)
    {
        while (sensor.key_2) reload_shift_reg();
        delay(6);

        if (speedStraight == 255)
        {
            speedStraight = 255;
            speedMiddle   = 248;
            speedCorner   = 218;
            speedRecover  = 188;
        }
        else
        {
            speedStraight = 255;
            speedMiddle   = 242;
            speedCorner   = 212;
            speedRecover  = 182;
        }
    }

    if (running)
        followTrackMaxPlus();
    else
        driveLR(0, 0);
}
