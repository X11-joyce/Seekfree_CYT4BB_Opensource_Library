#include "zf_common_headfile.h"
#include "pid.h"


//全局变量


//速度环 位置式pid
PID_TypeDef_t speed_pid;

//方向环外环 位置式pid
PID_TypeDef_t position_dir_speed_pid;

//方向环内环 位置式pid
PID_TypeDef_t position_angle_speed_pid; 

//角速度环
PID_TypeDef_t angle_speed_pid; 

//角度环
PID_TypeDef_t dir_angle_speed_pid; 

/*
*@brief  位置式pid初始化
*
*@note   输入0-10000的整数数值  GPIO_HIGH和GPIO_LOW分别控制电机的正转和反转 
*
*@param  PID_TypeDef_t结构体 kp ki kd max_output min_output  pd控制不使用i
*
*@return void
*
*/

void PID_Init(PID_TypeDef_t* pid,float Kp, float Ki, float Kd ,int max_output,int min_output )

{
    // 设置参数
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->last_output = 0;

    // 设置限幅值
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->max_integral = 0;

}

/*
*@brief  增量式pid初始化
*
*@note    
*
*@param  int形 （0-10000）的范围
*
*@return void
*
*/
void PID_Inc_Init(PID_Inc_TypeDef* pid, float Kp, float Ki, float Kd,int max_output,int min_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // 初始化状态
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->last_output = 0;

    // 默认限幅（使用时需手动设置）
    pid->max_output = max_output;
    pid->min_output = min_output;
}



/*
*@brief  增量式pid计算函数
*
*@note  核心公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
*
*@param  int形 （0-10000）的范围
*
*@return float类型，需要强制转换为int
*
*/
float PID_Inc_Compute(PID_Inc_TypeDef* pid, float setpoint, float input) {
    float error = setpoint - input;   
    float delta_output = pid->Kp * (error - pid->prev_error)
                       + pid->Ki * error
                       + pid->Kd * (error - 2*pid->prev_error + pid->prev_prev_error);

    // 计算新输出 = 上次输出 + 增量
    float output = pid->last_output + delta_output;

    // 输出限幅
    if (pid->max_output != pid->min_output) { 
        if (output > pid->max_output) {
            output = pid->max_output;
        } else if (output < pid->min_output) {
            output = pid->min_output;
        }
    }

    // 更新状态（注意顺序！）
    pid->prev_prev_error = pid->prev_error; // 先保存旧值
    pid->prev_error = error;
    pid->last_output = output;

    return output;
}


/*
*@brief  位置式pid计算函数
*
*@note  
*
*@param  PID_TypeDef_t  目标值  当前值
*
*@return void
*
*/
float PID_Position(PID_TypeDef_t* pid,float setpoint, float input)
{

    float error = ((float)setpoint - input);
    float P_out =(float) (pid->Kp * error);
    pid->integral += error;
    // 积分限幅
    //if (pid->integral > pid->max_integral) {
    //    pid->integral = pid->max_integral;
   // } else if (pid->integral < -pid->max_integral) {
   //     pid->integral = -pid->max_integral;
   // }
    float I_out = pid->Ki * pid->integral;
    // 微分项
    float D_out = pid->Kd * (error - pid->prev_error);
    // 计算总输出
    float output = P_out + I_out + D_out;
    // 输出限幅
   if (output > pid->max_output) {
       output = pid->max_output;
   } else if (output < pid->min_output) {
       output = pid->min_output;
   }
    // 保存状态
    pid->prev_error = error;
    pid->last_output = output;

    return output;
}


/*
*@brief  pid完整控制周期函数
*
*@note  在中断进行调用  10ms进行调用   速度环20ms周期  方向环10ms周期  速度环并方向环   后期调整为速度环并方向环
        应该先调试速度环 再调方向环 
*
*@param  
*
*@return void
*
*/


int16 SPEED_target_v = 50 ;  //期望速度
void pid_control_run(void)
{
    static int timer_flag = 0;  //用于记录周期
    static float L_pwm=0;   //左电机最终输出pwm
    static int R_pwm=0;   //右电机最终输出pwm
    static float dir_speed_out=0;
    float dir_angle_out;  //方向环最终输出量
    static float angle_speed_out=0;
    int speed_out = 0;
    float pitch_mid = 0.0;
   
/*
转向环  串级pid 外环图像处理  内环角速度
内环1ms 角速度   外环 图像误差10ms

为什么要用串级pid 外环视觉输出的是希望车需要摆正回来的量 作用到车上就是差速或者说角速度  内环再利用角速度进行修正
*/
    // if(0 == timer_flag % 10)//转向外环10ms    
    // {

    //     dir_speed_out =  PID_Position(&dir_speed_pid,0,0);  //到时再传入图像偏差
    // }
    // //内环
    // dir_angle_out = PID_Position(&angle_speed_pid,dir_speed_out,imu660ra_gyro_z);

/****************************************/
//速度环 20ms周期用于控制速度

/******************************************/
    if(0 == timer_flag % 20)//速度环 20ms
    {

       L_pwm = -PID_Position(&speed_pid,0,small_driver_value.receive_left_speed_data);
       
    }
     ik_right.theta = 1.56+L_pwm;
     ik_left.theta = 1.56-L_pwm;
     ik_right.r = 0.06;
     ik_left.r = 0.06;

/*
角度环 串角速度环
10ms 角度环
1ms 角速度环

*/
        if(0 == timer_flag % 10)
        {
            angle_speed_out = -PID_Position(&dir_angle_speed_pid,pitch_mid,euler_angle.pitch);

        }
        speed_out = (int)PID_Position(&angle_speed_pid,angle_speed_out,imu660rb_gyro_y);
       //printf("imu660rb gyro data: %f\n", euler_angle.pitch);
       // printf("imu660rb acc data:  %d\n", speed_out );
      small_driver_set_duty(&small_driver_value,speed_out,-speed_out);
   // small_driver_set_duty(&small_driver_value,1000,1000);
    timer_flag ++;

}











/*************************角度环pid***********************/
/***********位置式pid计算*************/
// float PID_Compute_3(PID_TypeDef_err* pid,float setpoint, float input)
// {

//     float error = setpoint - input;
//     float P_out = (pid->Kp * error) + (pid->Kp2 *abs( error )*error);
//     pid->integral += error;

//     // 积分限幅
//     //if (pid->integral > pid->max_integral) {
//     //    pid->integral = pid->max_integral;
//    // } else if (pid->integral < -pid->max_integral) {
//    //     pid->integral = -pid->max_integral;
//    // }
//     //float I_out = pid->Ki * pid->integral;
//     // 微分项

//     float D_out = (pid->Kd * (float)(-imu660ra_gyro_z))+pid->Kd2*(error - pid->prev_error) ;
//     // 计算总输出
//     float output = P_out  + D_out;
//     // 输出限幅
//    // if (output > pid->max_output) {
//     //    output = pid->max_output;
//    // } else if (output < pid->min_output) {
//    //     output = pid->min_output;
//    // }
//     // 保存状态
//     pid->prev_error = error;
//     pid->last_output = output;

//     return output;
// }