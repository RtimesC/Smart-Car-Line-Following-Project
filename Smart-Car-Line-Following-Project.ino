#include "Adafruit_NeoPixel.h"  //彩色灯珠驱动
#include "comm.h"               //传感器数据读取
#include "motor.h"              //电机控制

#define PIN    4
#define NUMPIXELS 2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// 电机速度常量
#define BASE_SPEED    230    // 基础速度
#define TURN_SPEED    90     // 转弯速度差
#define MAX_SPEED     255    // 最大速度
#define MIN_SPEED     0      // 最小速度
#define BACK_SPEED    100    // 后退速度

// 巡线模式
enum LineFollowMode {
  MODE_IDLE,      // 空闲模式
  MODE_FOLLOW,    // 巡线模式
  MODE_ROTATE     // 旋转模式
};

LineFollowMode currentMode = MODE_IDLE;

void setup()
{
    Serial.begin(9600);
    shift_reg_init();    // 传感器初始化
    motor_init();        // 电机初始化
    pixels.begin();      // 彩色灯珠初始化
    
    // 启动提示：绿灯亮
    pixels.setPixelColor(0, pixels.Color(0, 100, 0));
    pixels.setPixelColor(1, pixels.Color(0, 100, 0));
    pixels.show();
    delay(500);
}

void loop()
{
    reload_shift_reg();  // 刷新传感器数据（必须）
    
    // ========== 碰撞开关处理（优先级最高，放在最前面） ==========
    
    // 左前碰撞开关
    if(sensor.switcher_front_left_1 || sensor.switcher_front_left_2)
    {
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.setPixelColor(1, pixels.Color(100, 0, 0));
        pixels.show();
        
        motor_set_PWM(BACK_SPEED, -BACK_SPEED);  // 后退并右转
        delay(300);
        motor_set_PWM(0, 0);
        delay(100);
        
        if(currentMode == MODE_IDLE)
        {
            pixels.setPixelColor(0, pixels.Color(0, 100, 0));
            pixels.setPixelColor(1, pixels.Color(0, 100, 0));
        }
        else
        {
            pixels.setPixelColor(0, pixels.Color(0, 0, 100));
            pixels.setPixelColor(1, pixels.Color(0, 0, 100));
        }
        pixels.show();
    }
    
    // 右前碰撞开关
    if(sensor.switcher_front_right_1 || sensor.switcher_front_right_2)
    {
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.setPixelColor(1, pixels.Color(100, 0, 0));
        pixels.show();
        
        motor_set_PWM(-BACK_SPEED, BACK_SPEED);  // 后退并左转
        delay(300);
        motor_set_PWM(0, 0);
        delay(100);
        
        if(currentMode == MODE_IDLE)
        {
            pixels.setPixelColor(0, pixels.Color(0, 100, 0));
            pixels.setPixelColor(1, pixels.Color(0, 100, 0));
        }
        else
        {
            pixels.setPixelColor(0, pixels.Color(0, 0, 100));
            pixels.setPixelColor(1, pixels.Color(0, 0, 100));
        }
        pixels.show();
    }
    
    // 左后碰撞开关
    if(sensor.switcher_back_left)
    {
        currentMode = MODE_ROTATE;
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.setPixelColor(1, pixels.Color(100, 0, 0));
        pixels.show();
        
        motor_step(150, 0, 48, 0);
        motor_step(0, 0);
        currentMode = MODE_IDLE;
        
        pixels.setPixelColor(0, pixels.Color(0, 100, 0));
        pixels.setPixelColor(1, pixels.Color(0, 100, 0));
        pixels.show();
    }
    
    // 右后碰撞开关
    if(sensor.switcher_back_right)
    {
        currentMode = MODE_ROTATE;
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.setPixelColor(1, pixels.Color(100, 0, 0));
        pixels.show();
        
        motor_step(0, 150, 0, 48);
        motor_step(0, 0);
        currentMode = MODE_IDLE;
        
        pixels.setPixelColor(0, pixels.Color(0, 100, 0));
        pixels.setPixelColor(1, pixels.Color(0, 100, 0));
        pixels.show();
    }
    
    // ========== 按键控制模式切换 ==========
    if(sensor.key_1)
    {
        while(sensor.key_1) reload_shift_reg();
        delay(20);
        
        if(currentMode == MODE_IDLE)
        {
            currentMode = MODE_FOLLOW;
            pixels.setPixelColor(0, pixels.Color(0, 0, 100));
            pixels.setPixelColor(1, pixels.Color(0, 0, 100));
            pixels.show();
            delay(300);
        }
        else if(currentMode == MODE_FOLLOW)
        {
            currentMode = MODE_IDLE;
            motor_set_PWM(0, 0);
            pixels.setPixelColor(0, pixels.Color(0, 100, 0));
            pixels.setPixelColor(1, pixels.Color(0, 100, 0));
            pixels.show();
        }
    }
    
    // ========== 巡线模式 ==========
    if(currentMode == MODE_FOLLOW)
    {
        lineFollow();
    }
    
    pixels.show();
    delay(10);
}

// 巡线控制函数
void lineFollow()
{
    int leftSpeed = BASE_SPEED;
    int rightSpeed = BASE_SPEED;
    
    if(sensor.ir_mid)
    {
        leftSpeed = BASE_SPEED;
        rightSpeed = BASE_SPEED;
        pixels.setPixelColor(0, pixels.Color(100, 100, 100));
        pixels.setPixelColor(1, pixels.Color(100, 100, 100));
    }
else if(sensor.ir_right_1 || sensor.ir_right_2 || sensor.ir_right_3)
{
    int turn_offset = 0;
    int current_base = BASE_SPEED;

    // --- 直角弯/急弯检测逻辑 ---
    if(sensor.ir_right_3) 
    {
        // 判定为直角弯或严重偏移
        // 1. 极高的转向增量，迫使计算结果出现负数
        turn_offset = TURN_SPEED + 180; 
        // 2. 大幅压低基础速度，抵消惯性
        current_base = BASE_SPEED - 120; 
    }
    else if(sensor.ir_right_2) 
    {
        turn_offset = TURN_SPEED + 50;
        current_base = BASE_SPEED - 40;
    }
    else // 只有 ir_right_1 亮
    {
        turn_offset = TURN_SPEED - 50; 
        current_base = BASE_SPEED; 
    }

    rightSpeed = current_base - turn_offset;
    leftSpeed  = current_base + turn_offset;

    // --- 直角弯核心控制：强制内轮反转 ---
    if(leftSpeed > 255) leftSpeed = 255;
    
    // 如果是直角弯(ir_right_3亮)，内轮必须给足负分分量实现原地自旋
    // 注意：如果你的电机库不支持负数，请将此处改为 rightSpeed = 0; 但反转效果最好
    if(rightSpeed < -180) rightSpeed = -180; 

    pixels.setPixelColor(0, pixels.Color(0, 0, 100));
    pixels.setPixelColor(1, pixels.Color(0, 100, 0));
}
else if(sensor.ir_left_1 || sensor.ir_left_2 || sensor.ir_left_3)
{
    int turn_offset = 0;
    int current_base = BASE_SPEED;

    if(sensor.ir_left_3) 
    {
        turn_offset = TURN_SPEED + 180;
        current_base = BASE_SPEED - 120;
    }
    else if(sensor.ir_left_2) 
    {
        turn_offset = TURN_SPEED + 50;
        current_base = BASE_SPEED - 40;
    }
    else 
    {
        turn_offset = TURN_SPEED - 50;
        current_base = BASE_SPEED;
    }

    leftSpeed  = current_base - turn_offset;
    rightSpeed = current_base + turn_offset;

    if(rightSpeed > 255) rightSpeed = 255;
    if(leftSpeed < -180) leftSpeed = -180; 

    pixels.setPixelColor(0, pixels.Color(0, 0, 100));
    pixels.setPixelColor(1, pixels.Color(0, 100, 0));
}

    else
    {
        leftSpeed = BASE_SPEED / 2;
        rightSpeed = -BASE_SPEED / 2;
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.setPixelColor(1, pixels.Color(100, 0, 0));
    }
    
    if(leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;
    if(leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
    if(rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;
    if(rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
    
    motor_set_PWM(leftSpeed, rightSpeed);
}
