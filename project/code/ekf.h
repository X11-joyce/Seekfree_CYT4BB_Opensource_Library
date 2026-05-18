#include "zf_common_headfile.h"
#ifndef CODE_EKF_H_
#define CODE_EKF_H_

#define DEG_TO_RAD      (57.295779513082320876798154814105f)        //角度转弧度
/*********************************************************************参数*********************************************************************/
extern float v_hat, x_hat;                                          //真实速度、位移
extern float acc_x_lpf;                                             //加速度低通滤波

extern float dt;                                                    //ekf更新周期
extern float dt_ekf;
extern float GY;
extern float acc_b[3];
typedef struct
{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
}imu_t;
extern imu_t imu_data;

/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
void EKF_Init(void);                                            //EKF初始化

void imu_get_values(void);                                      //得到imu原始值

void EKF_UpData(void);                                          //更新EKF数据

void SOSFilter(float *input, float *output, int length);        //Direct Form II 二阶节滤波

void EKF_V_UPData(void);                                        //更新EKF得到真实的位移、速度

float fast_invsqrt(float num);
/*********************************************************************函数*********************************************************************/


#endif /* CODE_EKF_H_ */