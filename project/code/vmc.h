#ifndef vmc_H_
#define vmc_H_
#include "zf_common_headfile.h"

#define PI 3.14159265358979323846f
//5连杆长度


#define L5 0.0375f   //37.5
#define L1 0.0546f   //54.16
#define L2 0.09025f   //90.25
#define L3 0.09025f
#define L4 0.054f

#define DEG2RAD     (PI/180.0f)   // 角度 → 弧度
#define RAD2DEG     (180.0f/PI)   // 弧度 → 角度
//腿骨关节的角度
typedef struct{

	float alpha;  //前 关节电机角度
	float beta;		//后
	

}JointAngle_t; 

//节点
typedef struct{

	float x;
	float y;

}Node;
//vmc 解算
typedef struct{
	
	float L0;		 //虚拟杆长	
	float phi0; //虚拟杆的角度
	
	float Tp;  //转矩 lqr输出
	float F0;  //支持力 
	
	float phi1; //前腿角度
	float phi2; //前腿中间坐标
	float phi3; //后腿中间角度
	float phi4; //后腿角度
	
	float torque_set[2]; //最终前后关节电机输出力矩
	
	JointAngle_t joint_angle ;  //前后关节电机的原始角度
	
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	
	
	float d_L0;
	float last_L0;
	float last_d_L0;
	float dd_L0;
	
	float dd_theta;
	float last_d_theta;

	float last_phi0 ;
	float alpha;
	float d_alpha;
	float d_phi0;
	float theta;
	float d_theta;
	
	
	Node D;
	Node C;
	Node B;
	uint8_t first_flag;
	
	
} VMC_t;


typedef struct
{
    float r;  //左腿关节角度
    float theta;  //左腿关节角度

	float alpha;  //前 关节电机角度
	float beta;		//后 关节电机角度
    float x;
    float y;

    int pwm_1;
    int pwm_2;

}IK_t;
extern IK_t ik_left;
extern IK_t ik_right;
extern VMC_t vmc_right;
extern VMC_t vmc_left;

void forward_kinematics_right(VMC_t* vmc,float dt);
void forward_kinematics_left(VMC_t* vmc,float dt);
void LegIKines_right(IK_t *ik);
void LegIKines_left(IK_t *ik);
#endif /* vmc_H_ */
