#include "stree_control.h"


Servo_control steer_1;
Servo_control steer_2;
Servo_control steer_3;
Servo_control steer_4;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介          误差限幅
// 返回类型          float
// 使用示例          constrain_float(pid->integrator, -pid->imax, pid->imax);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          舵机初始化
// 返回类型
// 使用示例          Servo_Init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void)
{
    steer_1.center = 4500;              //左后  顺时针转  1700(下)  4500 7500（上）
    steer_1.control_frequency = SERVO_FREQ;
    steer_1.pwm_pin = STEER_1_PWM;
    steer_1.steer_dir = 1;

    steer_2.center = 4500;              //右前  顺时针转  1700（下）  4500 7500（上）
    steer_2.control_frequency = SERVO_FREQ;
    steer_2.pwm_pin = STEER_2_PWM;
    steer_2.steer_dir = 1;

    steer_3.center = 4500;              //右后  顺时针转  1500（上）  4500 7300（下）
    steer_3.control_frequency = SERVO_FREQ;
    steer_3.pwm_pin = STEER_3_PWM;
    steer_3.steer_dir = 1;

    steer_4.center = 4500;             //左前  顺时针转  1500（上）  4500 7300（下）
    steer_4.control_frequency = SERVO_FREQ;
    steer_4.pwm_pin = STEER_4_PWM;
    steer_4.steer_dir = 1;


    steer_1.vale = steer_1.center ;
    steer_2.vale = steer_2.center ;
    steer_3.vale = steer_3.center ;
    steer_4.vale = steer_4.center ;
    
    
    steer_1.steer_angle = 180 - (steer_1.center - 1500) * 180 / 6000;
    steer_2.steer_angle = 180 - (steer_2.center - 1500) * 180 / 6000;
    steer_3.steer_angle = (steer_3.center - 1500.0f) * 180.0f / 6000.0f;
    steer_4.steer_angle = (steer_4.center - 1500.0f) * 180.0f / 6000.0f;

    pwm_init(steer_1.pwm_pin, SERVO_FREQ, steer_1.center);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(steer_2.pwm_pin, SERVO_FREQ, steer_2.center);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(steer_3.pwm_pin, SERVO_FREQ, steer_3.center);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(steer_4.pwm_pin, SERVO_FREQ, steer_4.center); 


}



//用于pwm数值与与角度转换
void servo2angle()
{
    steer_1.steer_angle = 180 - (steer_1.vale  - 1500) * 180 / 6000;
    steer_2.steer_angle = 180 - (steer_2.vale - 1500) * 180 / 6000;
    steer_3.steer_angle = (steer_3.vale - 1500.0f) * 180.0f / 6000.0f;
    steer_4.steer_angle = (steer_4.vale - 1500.0f) * 180.0f / 6000.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          舵机限制占空比范围
// 返回类型
// 使用示例          ClampDuty(-100,100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 ClampDuty(uint32 duty, uint32 min, uint32 max) {
    if (duty > max) return max;
    if (duty < min) return min;
    return duty;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介          舵机控制函数  1 位置控制 直接控制pwm   传入4个舵机的角度值，进行映射到pwm输出 直接控制位置
// 返回类型
// 使用示例          Servo_Control(100,100,100,100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Servo_Control_1(float angle_1,float angle_2,float angle_3,float angle_4)//float angle_1,,float angle_4
{
    int ch1 = 7500 - (int)(angle_1 * 33.3333f); // 角度转占空比 0-180度对应7500-1500占空比
    int ch2 = 7500 - (int)(angle_2 * 33.3333f); // 角度转占空比 0-180度对应7500-1500占空比


    int ch3 = (int)angle_3*33.3333f + 1500; // 角度转占空比 0-180度对应1500-7500占空比
    int ch4 = (int)angle_4*33.3333f + 1500; // 角度转占空比 0-180度对应1500-7500占空比

    steer_1.vale=ClampDuty(ch1,1500,7500);
    steer_2.vale=ClampDuty(ch2,1500,7500);
    steer_3.vale=ClampDuty(ch3,1500,7500);
    steer_4.vale=ClampDuty(ch4,1500,7500);
    servo2angle(); //更新角度值

    pwm_set_duty(steer_1.pwm_pin,steer_1.vale);    //左后
    pwm_set_duty(steer_2.pwm_pin,steer_2.vale);    //右前
    pwm_set_duty(steer_3.pwm_pin,steer_3.vale);   //右后
    pwm_set_duty(steer_4.pwm_pin,steer_4.vale);   //左前
}