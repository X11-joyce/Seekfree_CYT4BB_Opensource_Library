#ifndef __PID_H
#define __PID_H

//全局变量

//位置式pid结构体
typedef struct 
{
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float max_output;   // 输出上限
    float min_output;   // 输出下限
    float max_integral; // 积分限幅

        // 运行时状态
    float integral;     // 积分累计值
    float prev_error;   // 上一次误差
    float last_output;  // 上一次输出
    int period;  //周期

    
}PID_TypeDef_t;


typedef struct {
    float Kp;           // 比例系数

    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float max_output;   // 输出上限
    float min_output;   // 输出下限
    float max_integral; // 积分限幅

    // 运行时状态（增量式PID需要保存前两次的误差）
    float prev_error;      // 上一次误差 e(k-1)
    float prev_prev_error; // 上上次误差 e(k-2)
    float last_output;     // 上一次输出 u(k-1)
    int period;  //周期

} PID_Inc_TypeDef;


extern PID_TypeDef_t speed_pid;

//方向环外环 位置式pid
extern PID_TypeDef_t position_dir_speed_pid;

//方向环内环 位置式pid
extern PID_TypeDef_t position_angle_speed_pid; 
extern int16 SPEED_target_v  ;  //期望速度

//角度环 角速度环
extern PID_TypeDef_t angle_speed_pid; 
extern PID_TypeDef_t dir_angle_speed_pid; 
/*******函数******** */

void PID_Init(PID_TypeDef_t* pid,float Kp, float Ki, float Kd ,int max_output,int min_output );

void PID_Inc_Init(PID_Inc_TypeDef* pid, float Kp, float Ki, float Kd,int max_output,int min_output);

float PID_Inc_Compute(PID_Inc_TypeDef* pid, float setpoint, float input);

float PID_Position(PID_TypeDef_t* pid,float setpoint, float input);

void pid_control_run(void);


#endif // !__PID_H