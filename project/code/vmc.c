//运动学解算

#include "zf_common_headfile.h"

VMC_t vmc_right;
VMC_t vmc_left;

/*******正运动学计算函数***********/
void forward_kinematics_right(VMC_t* vmc,float dt)
{
	float theta1,theta2 ,selectedtheta;
	
	static float PitchR=0.0f;
	static float PithGyroR=0.0f;
    PitchR=euler_angle.pitch;
	PithGyroR=imu660rb_gyro_y;
	
	/******ph1 ph4的角度******/
	//右腿  关节的角度 
	vmc->joint_angle.alpha = steer_1.steer_angle;//前 弧度 E   填入舵机的角度   右前 4500 为0度
	vmc->joint_angle.beta = steer_2.steer_angle;//后 弧度 A             右后 4500 为180度
	
	//(机械零点时 前腿为180  后腿为0  ) 逆时针为正方向 右腿坐标系
	vmc->phi4 = (steer_2.steer_angle-90.f)  * DEG2RAD;   //前腿
	vmc->phi1 = (180-steer_3.steer_angle+90.f) * DEG2RAD; //后腿

	/******ph1 ph4的角度******/
	
	
	/******中间B,D坐标******/
	vmc->D.y = L4*sin(vmc->phi4);
	vmc->D.x = L5 +L4 *cos(vmc->phi4);
	
	vmc->B.y = L1 *sin(vmc->phi1);
	vmc->B.x = L1 *cos(vmc->phi1);
	/******中间B,D坐标******/
	
	//中间信息 a  c坐标与末端坐标的夹角
	float lright = sqrt((vmc->D.x  - vmc->B.x)*(vmc->D.x  - vmc->B.x)+(vmc->D.y - vmc->B.y)*(vmc->D.y - vmc->B.y));
	float aright = 2*(vmc->D.x -vmc->B.x)*L2;
    float bright = 2*(vmc->D.y -vmc->B.y)*L2;	
	float cright = (L2*L2 +lright*lright-L3*L3 );
	

	theta1 =2 *atan2f((bright+sqrt(aright*aright+bright*bright-cright*cright)),aright+cright); //2 *atan((bright +sqrt((bright * bright) + (aright *aright)-(cright *cright)))/(aright+cright));
    vmc->phi2 = theta1;
	
//	theta2 = atan2f((vmc->B.y-vmc->D.y+L2*sin(theta1)),vmc->B.x-vmc->D.x+L2*cos(theta1));
//  vmc->phi3 = theta2;
	//中间信息 a  c坐标与末端坐标的夹角
	
	
	
	//C点直角坐标
	vmc->C.x =  L1*cos(vmc->phi1)+L2*cos(theta1); //         vmc->A.x  +L2 *cos(selectedtheta);
	vmc->C.y =  L1*sin(vmc->phi1)+L2*sin(theta1);   //            vmc->A.y +L2 *sin(selectedtheta);


		//虚拟杆长
	vmc->L0 = sqrt((vmc->C.x-L5/2)*(vmc->C.x-L5/2)+(vmc->C.y *vmc->C.y ));
	
	vmc->phi0 = atan2f(vmc->C.y ,(vmc->C.x-L5/2.0f));  

	vmc->alpha=PI/2.0f-vmc->phi0 ;

	
	if(vmc->first_flag==0)
	{
		vmc->last_phi0=vmc->phi0 ;
		vmc->first_flag=1;
	}
	
	vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta	
	vmc->d_alpha=0.0f-vmc->d_phi0 ;
	
	vmc->theta=PI/2.0f+PitchR - vmc->phi0;//得到机体状态变量1	
	vmc->d_theta=(PithGyroR-vmc->d_phi0);//得到机体状态变量2

	vmc->last_phi0=vmc->phi0 ;
	
	vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
	vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
	
	vmc->last_d_L0=vmc->d_L0;
	vmc->last_L0=vmc->L0;
	
	vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
	vmc->last_d_theta=vmc->d_theta;	


}



void forward_kinematics_left(VMC_t* vmc,float dt)
{
	float theta1,theta2 ,selectedtheta;
	
	static float PitchR=0.0f;
	static float PithGyroR=0.0f;

	//左腿  关节的角度
	vmc->joint_angle.alpha = steer_3.steer_angle;	//前
	vmc->joint_angle.beta = steer_4.steer_angle;	//后
	
	vmc->phi4 = (steer_1.steer_angle-90.f)  * DEG2RAD;		//(机械零点时 前腿为0  后腿为180  )
	vmc->phi1 = (180.f- steer_4.steer_angle+90.f) * DEG2RAD;
	
	
	//得到中间A,C坐标
	vmc->D.y = L4*sin(vmc->phi4);
	vmc->B.y = L1 *sin(vmc->phi1);
	vmc->D.x = L5 +L4 *cos(vmc->phi4);
	vmc->B.x = L1 *cos(vmc->phi1);
	
	float lright = sqrt((vmc->D.x  - vmc->B.x)*(vmc->D.x  - vmc->B.x)+(vmc->D.y - vmc->B.y)*(vmc->D.y - vmc->B.y));
	
	//中间信息 a  c坐标与末端坐标的夹角
	float aright = 2*(vmc->D.x -vmc->B.x)*L2;
    float bright = 2*(vmc->D.y -vmc->B.y)*L2;	
	float cright = (L2*L2 +lright*lright-L3*L3 );
	
	//得到大腿关节的角度
	theta1 =2 *atan2f((bright+sqrt(aright*aright+bright*bright-cright*cright)),aright+cright); //2 *atan((bright +sqrt((bright * bright) + (aright *aright)-(cright *cright)))/(aright+cright));
    vmc->phi2 = theta1;
	
	theta2 = atan2f((vmc->B.y-vmc->D.y+L2*sin(theta1)),vmc->B.x-vmc->D.x+L2*cos(theta1));
    vmc->phi3 = theta2;
	
	
	vmc->C.x =  L1*cos(vmc->phi1)+L2*cos(theta1); //         vmc->A.x  +L2 *cos(selectedtheta);
	vmc->C.y =  L1*sin(vmc->phi1)+L2*sin(theta1);   //            vmc->A.y +L2 *sin(selectedtheta);


		//虚拟杆长
	vmc->L0 = sqrt((vmc->C.x-L5/2)*(vmc->C.x-L5/2)+(vmc->C.y *vmc->C.y ));
	vmc->phi0 = atan2f(vmc->C.y ,(vmc->C.x-L5/2.0f));
  
	vmc->alpha=PI/2.0f-vmc->phi0 ;

	
	if(vmc->first_flag==0)
	{
		vmc->last_phi0=vmc->phi0 ;
		vmc->first_flag=1;
	}
	vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta	
	vmc->d_alpha=0.0f-vmc->d_phi0 ;
	
	vmc->theta=PI/2.0f+PitchR - vmc->phi0;//得到机体状态变量1	
	vmc->d_theta=(PithGyroR-vmc->d_phi0);//得到机体状态变量2

	vmc->last_phi0=vmc->phi0 ;
	
	vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
	vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
	
	vmc->last_d_L0=vmc->d_L0;
	vmc->last_L0=vmc->L0;
	
	vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
	vmc->last_d_theta=vmc->d_theta;	

}



///*******逆运动学计算函数***********/

//使用极坐标作为输入，输出舵机角度


void PolarToCartesian(IK_t *ik) {
    ik->x = L5/2 + ik->r*cosf(ik->theta);
    ik->y = ik->r*sin(ik->theta);
    
    
}
//这个地方存在镜像处理  右腿坐标系与左腿坐标系相反  需要对输入的theta进行处理   right 向前 需要增大  left 向前需要减小
IK_t ik_left = {
    .r = 0.06f,
    .theta = 1.56f,
    .alpha = 0.0f,
    .beta = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .pwm_1 = 0,
    .pwm_2 = 0
};
IK_t ik_right = {
    .r = 0.06f,
    .theta = 1.56f,
    .alpha = 0.0f,
    .beta = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .pwm_1 = 0,
    .pwm_2 = 0
};

void LegIKines_right(IK_t *ik)
{
    
    float Bx, By, Dx, Dy;
    float phi1, phi4, phi2, phi3;
    float a, b, c,d, e, f;
	PolarToCartesian(ik);
	a=2*ik->x*L1;
	b=2*ik->y*L1;
	c=ik->x*ik->x+ik->y*ik->y+L1*L1-L2*L2;
	d=2*L4*(ik->x-L5);
	e=2*L4*ik->y;
	f=(ik->x-L5)*(ik->x-L5)+ik->y*ik->y+L4*L4-L3*L3;
	
	float sqrt_val1 = sqrtf(a*a + b*b - c*c);
    phi1 = atan2f(b, a) + atan2f(sqrt_val1, c);
    phi4 = atan2f(e, d) - atan2f(sqrtf(d*d + e*e - f*f), f);

	Bx = L1 * cosf(phi1);
    By = L1 * sinf(phi1);
    Dx = L5 + L4 * cosf(phi4);
    Dy = L4 * sinf(phi4);

    phi2 = atan2f(ik->y - By, ik->x - Bx) - phi1;
    phi3 = atan2f(ik->y - Dy, ik->x - Dx) - phi4;
   
    //printf("angle: %f\n", vmc_right.phi0);

    // ================= 最终输出到你的结构体 =================
    ik->alpha = (phi4*RAD2DEG)+90.f;    // 前关节角度
     
    ik->beta  = 270-(phi1*RAD2DEG);    // 后关节角度
    //Servo_Control_1(ik->alpha, ik->beta);
    //printf("angle: %f\n", ik->x);

}

void LegIKines_left(IK_t *ik)
{
    
    float Bx, By, Dx, Dy;
    float phi1, phi4, phi2, phi3;
    float a, b, c,d, e, f;
	PolarToCartesian(ik);
	a=2*ik->x*L1;
	b=2*ik->y*L1;
	c=ik->x*ik->x+ik->y*ik->y+L1*L1-L2*L2;
	d=2*L4*(ik->x-L5);
	e=2*L4*ik->y;
	f=(ik->x-L5)*(ik->x-L5)+ik->y*ik->y+L4*L4-L3*L3;
	
	float sqrt_val1 = sqrtf(a*a + b*b - c*c);
    phi1 = atan2f(b, a) + atan2f(sqrt_val1, c);
    phi4 = atan2f(e, d) - atan2f(sqrtf(d*d + e*e - f*f), f);

	Bx = L1 * cosf(phi1);
    By = L1 * sinf(phi1);
    Dx = L5 + L4 * cosf(phi4);
    Dy = L4 * sinf(phi4);

    phi2 = atan2f(ik->y - By, ik->x - Bx) - phi1;
    phi3 = atan2f(ik->y - Dy, ik->x - Dx) - phi4;
   
    //printf("angle: %f\n", vmc_right.phi0);

    // ================= 最终输出到你的结构体 =================
    ik->alpha = (phi4*RAD2DEG)+90.f;    // 前关节角度
     
    ik->beta  = 270-(phi1*RAD2DEG);    // 后关节角度
    //Servo_Control_1(ik->beta, ik->alpha);
    //printf("angle: %f\n", ik->x);

}


void IK_control(IK_t *ik_right, IK_t *ik_left)
{
    LegIKines_right(ik_right);
    LegIKines_left(ik_left);
    Servo_Control_1(ik_left->alpha, ik_right->alpha, ik_right->beta, ik_left->beta);
}