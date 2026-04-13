#include "Adafruit_NeoPixel.h"  //彩色灯珠驱动
#include "comm.h"               //传感器数据读取
#include "motor.h"              //电机控制

#define PIN            4
#define NUMPIXELS      2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() 
{
    shift_reg_init();   //传感器初始化
    motor_init();       //电机初始化
    pixels.begin();     //彩色灯珠初始化
}
void loop()                                                                             
{
    static int motor_right;
    static int motor_left;

    char r0,r1,g0,g1,b0,b1;
    reload_shift_reg(); //刷新传感器数据
    
    if(sensor.ir_left_3) {//左侧第三个(最外面)红外传感器测到黑线或悬空
        g0 = 100; //设定第1个灯珠亮红色
        r0=r1=g1=b0=b1 = 0;
        motor_left = 0;
        motor_right = 255; 


    }
    else if(sensor.ir_left_2) {
        r0 = 100;  //g红色  r绿色  b蓝色
        r1=g0=g1=b0=b1 = 0;
        motor_left = 100;
        motor_right = 255; 
    }
    else if(sensor.ir_left_1) {
        b0 = 100; 
        r0=r1=g0=g1=b1 = 0;
        motor_left = 150;
        motor_right = 255; 
    }
    else if(sensor.ir_right_1) {
        b1 = 100; 
        r0=r1=g0=g1=b0 = 0;
        motor_left = 255;
        motor_right = 150; 
    }
    else if(sensor.ir_right_2) {
        r1 = 100; 
        r0=g0=g1=b0=b1 = 0;
        motor_left = 255;
        motor_right = 100; 
    }
    else if(sensor.ir_right_3) {
        g1 = 100; 
        r0=r1=g0=b0=b1 = 0;
        motor_left = 255;
        motor_right = 0;
    }
    else if(sensor.ir_mid) {
        r0=r1=g0=g1=b0=b1 = 100;
        motor_left = 255;
        motor_right = 255;
    }
        
    pixels.setPixelColor(0, pixels.Color(r0,g0,b0)); //设定第一个灯珠颜色RGB(0~255)
    pixels.setPixelColor(1, pixels.Color(r1,g1,b1)); //设定第二个灯珠颜色
    pixels.show();  //显示设定好的颜色
motor_set_PWM(motor_left,motor_right); //设定电机速度(左,右)(0~255)
delay(20);
}
