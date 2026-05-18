#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"
//*****************宏定义****************//


#define SERVO_FREQ 300
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))



#define STEER_1_PWM   (TCPWM_CH09_P05_0)

#define STEER_2_PWM   (TCPWM_CH12_P05_3)

#define STEER_3_PWM   (TCPWM_CH11_P05_2)

#define STEER_4_PWM   (TCPWM_CH10_P05_1)


//*****************结构体***************//

typedef struct
{
    pwm_channel_enum  pwm_pin;   //PWM引脚
    int16 control_frequency;    //控制频率
    int16 steer_dir;            //舵机转向 1为正转 -1为反转
    float steer_angle;          //舵机转角
    int16 center;               //中心
    int16 vale;                 //当前值
}Servo_control;



extern Servo_control steer_1;
extern Servo_control steer_2;   
extern Servo_control steer_3;
extern Servo_control steer_4;


//*****************函数声明***************//
void Servo_Init(void);                                            //舵机初始化
void Servo_Control_1(float angle_1,float angle_2,float angle_3,float angle_4);
#endif