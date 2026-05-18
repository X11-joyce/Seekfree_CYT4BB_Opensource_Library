#include "zf_common_headfile.h"


#define ikun            (0.00215885305953045270200687624638f)

matrix_t exf_x;
matrix_t error_1;
EulerAngles euler_angle;
imu_t imu_data = {0, 0, 0, 0, 0, 0};
matrix_type r_yz = 0.01f;

float acc_b[3] = {0, 0, 9.8};

//调节矩阵, 控制姿态结算速度
const matrix_type q[4][4] = {{0.007, 0, 0, 0}, {0, 0.007, 0, 0}, {0, 0, 0.007, 0}, {0, 0, 0, 0.007}};
const matrix_type r[3][3] = {{100000, 0, 0}, {0, 100000, 0}, {0, 0, 100000}};
const matrix_type p[4][4] = {{1000000, 0, 0, 0}, {0, 1000000, 0, 0}, {0, 0, 1000000, 0}, {0, 0, 0, 1000000}};
const matrix_type ekf[4] = {1, 0, 0, 0};

//姿态解算的Q、R、P矩阵
static matrix_t Q;
static matrix_t R;
static matrix_t P;

//加速度低通滤波
static int16 imu660rb_acc_x_l = 0;
static int16 imu660rb_acc_y_l = 0;
static int16 imu660rb_acc_z_l = 0;

static float PK[4] = {1000, 100, 100, 1000};
static float Kk[2] = {0, 0};
float Q_ekf[4] = {0.3, 0.001, 0.001, 0.2};
float R_ekf = 0.1;
float v_hat = 0;
float x_hat = 0;

float acc_x_lpf = 0;
const float K_V = 0.05;


//EKF运行周期
float dt = 0.001f;
float dt_ekf = 0.01f;


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     EKF初始化
// 参数说明     null
// 返回参数     null
// 使用示例     EKF_Init();
// 备注信息     拓展卡尔曼滤波初始化, 总初始化调用
-------------------------------------------------------------------------------------------------------------------*/
void EKF_Init(void)
{
    Matrix_From_Array(&exf_x, (const matrix_type*)ekf, 4, 1);
    Matrix_From_Array(&Q, (const matrix_type*)q, 4, 4);
    Matrix_From_Array(&R, (const matrix_type*)r, 3, 3);
    Matrix_From_Array(&P, (const matrix_type*)p, 4, 4);

    imu660ra_get_acc();
    float ax = imu660rb_acc_x;
    float ay = imu660rb_acc_y;
    float az = imu660rb_acc_z;
    float norm = fast_invsqrt((float)ax * ax + ay * ay + az * az);

    ax *= norm;
    ay *= norm;
    az *= norm;

    float pitch = asinf(ax);                // 俯仰角
    float roll = atan2f(ay, az);            // 滚转角

    // 计算四元数
    float cy = cosf(roll * 0.5f);
    float sy = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    exf_x.data[0][0] = cy * cp;
    exf_x.data[1][0] = cy * sp;
    exf_x.data[2][0] = sy * cp;
    exf_x.data[3][0] = sy * sp;
    normalize_vector(&exf_x);
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角
// 参数说明     null
// 返回参数     static inline void
// 使用示例     quaternion_to_euler();
// 备注信息     无
-------------------------------------------------------------------------------------------------------------------*/
static inline void quaternion_to_euler(void)
{
    const float G = 9.8;
    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);
    float ax = imu660ra_acc_transition(imu_data.acc_x) * 9.8f;
    float ay = imu660ra_acc_transition(imu_data.acc_y) * 9.8f;
    float az = imu660ra_acc_transition(imu_data.acc_z) * 9.8f;
    acc_b[0] = ax - (2 * G * q1 * q3 - 2 * G * q0 * q2);
    acc_b[1] = ay - (2 * G * q0 * q1 + 2 * G * q2 * q3);
    acc_b[2] = az - (G * q0 * q0 - G * q1 * q1 - G * q2 * q2 + G * q3 * q3);
    euler_angle.pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * DEG_TO_RAD;                                  // pitch
    euler_angle.roll = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * DEG_TO_RAD;   // roll
    euler_angle.yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * DEG_TO_RAD;    // yaw
}


#define K               (1.0f)                                      //加速度低通滤波系数
/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     得到imu原始值
// 参数说明     null
// 返回参数     null
// 使用示例     imu_get_values();
// 备注信息     这里得到的是真实物理量
-------------------------------------------------------------------------------------------------------------------*/
void imu_get_values(void)
{
    imu660rb_get_gyro();
    imu660rb_get_acc();

    //一阶低通滤波，单位g/s
    imu_data.acc_x = K *  (imu660rb_acc_x)  + (1 - K) * imu660rb_acc_x_l;
    imu_data.acc_y = K *  (imu660rb_acc_y)  + (1 - K) * imu660rb_acc_y_l;
    imu_data.acc_z = K *  (imu660rb_acc_z)  + (1 - K) * imu660rb_acc_z_l;
    imu660rb_acc_x_l = imu_data.acc_x;
    imu660rb_acc_y_l = imu_data.acc_y;
    imu660rb_acc_z_l = imu_data.acc_z;

    //陀螺仪角度转弧度
    imu_data.gyro_x = (imu660rb_gyro_x) * PI / 180 / 16.384f;
    imu_data.gyro_y = (imu660rb_gyro_y) * PI / 180 / 16.384f;
    imu_data.gyro_z = (imu660rb_gyro_z) * PI / 180 / 16.384f;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新EKF数据
// 参数说明     null
// 返回参数     null
// 使用示例     EKF_UpData();
// 备注信息     运行拓展卡尔曼滤波
-------------------------------------------------------------------------------------------------------------------*/
void EKF_UpData(void)
{

    float gx, gy, gz;
    imu_get_values();
    gx = imu_data.gyro_x;
    gy = imu_data.gyro_y;
    gz = imu_data.gyro_z;

    matrix_t Z;

    Matrix_Init(&Z, 3, 1);

    Z.data[0][0] = (matrix_type)imu_data.acc_x;
    Z.data[1][0] = (matrix_type)imu_data.acc_y;
    Z.data[2][0] = (matrix_type)imu_data.acc_z;

    normalize_vector(&Z);
    matrix_type f[4][4]= {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                          {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                          {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                          {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};

    matrix_t F,FT;
    Matrix_From_Array(&F, (const matrix_type*)f, 4, 4);
    FT = Matrix_Transpose(&F);

    exf_x = multiply_matrices(&F, &exf_x);  //X = F * X;
    normalize_vector(&exf_x);

    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    matrix_type h[3][4]={{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
                         {2 * q1, 2 * q0, 2 * q3, 2 * q2},
                         {2 * q0, -2 * q1, -2 * q2, 2 * q3}};

    matrix_t H, HT;
    Matrix_From_Array(&H, (const matrix_type*)h, 3, 4);
    HT = Matrix_Transpose(&H);
    matrix_t PK_;

    //PK_ = F * P(K - 1) * FT + Q;
    PK_ = multiply_matrices(&F, &P);       //F * P;
    PK_ = multiply_matrices(&PK_, &FT);    //F * P * FT;
    P = add_matrices(&PK_, &Q);            //F * P * FT + Q;


    //DK_ = H * PK_ * HT + R;
    matrix_t DK, invDK;
    DK = multiply_matrices(&H, &P);
    DK = multiply_matrices(&DK, &HT);
    DK = add_matrices(&DK, &R);

    if(inverse_matrix(&DK, &invDK))
    {
        quaternion_to_euler();
        return;
    }

    //ek = Z - H * X;
    matrix_t EK, EKT;
    EK = multiply_matrices(&H, &exf_x);     //H * X;
    EK = subtract_matrices(&Z, &EK);        //Z - HX;
    EKT = Matrix_Transpose(&EK);

    //r = EKT * invDK * EK;
    error_1 = multiply_matrices(&EKT, &invDK);
    error_1 = multiply_matrices(&error_1, &EK);

    if(error_1.data[0][0] > r_yz)
    {
        quaternion_to_euler();
        return;
    }

    //Kk = M * P * HT * invDK;
    matrix_t Kk;
    Kk = multiply_matrices(&P, &HT);
    Kk = multiply_matrices(&Kk, &invDK);

    //X = X_ + Kk * Ek;
    matrix_t temp;
    temp = multiply_matrices(&Kk, &EK);
    exf_x = add_matrices(&exf_x, &temp);
    normalize_vector(&exf_x);

    //P = (I - Kk * H) * PK_;
    matrix_t I;
    Matrix_Identity(&I, 4);
    temp = multiply_matrices(&Kk, &H);
    temp = subtract_matrices(&I, &temp);
    P = multiply_matrices(&temp, &P);
    quaternion_to_euler();
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新EKF得到真实的位移、速度
// 参数说明     null
// 返回参数     null
// 使用示例     EKF_V_UPData();
// 备注信息     无
-------------------------------------------------------------------------------------------------------------------*/
float GY;
void EKF_V_UPData(void)
{
    float a = -(acc_b[0] * cosf(euler_angle.pitch / DEG_TO_RAD) - 0.0195);
    GY = -imu660ra_gyro_transition(imu660ra_gyro_y) / DEG_TO_RAD;
    //先验估计
    x_hat = x_hat + v_hat * dt_ekf;// + 0.5f * a * dt_ekf * dt_ekf;
    v_hat = v_hat + a * dt_ekf;

    //计算Pk
    PK[0] = PK[0] + dt_ekf * PK[2] + dt_ekf * (PK[1] + dt_ekf * PK[3]) + Q_ekf[0];
    PK[1] = PK[1] + dt_ekf * PK[3] + Q_ekf[1];
    PK[2] = PK[2] + dt_ekf * PK[3] + Q_ekf[2];
    PK[3] = PK[3] + Q_ekf[3];

    //计算Kk
    Kk[0] = PK[1] / (PK[3] + R_ekf);
    Kk[1] = PK[3] / (PK[3] + R_ekf);

    //更新X
    //float v = (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) * ikun;
    //x_hat = x_hat + Kk[0] * (v - v_hat);
    //v_hat = v_hat + Kk[1] * (v - v_hat);

    //更新PK
    PK[0] = PK[0] - Kk[0] * PK[2];
    PK[1] = PK[1] - Kk[0] * PK[3];
    PK[2] = -PK[2] * (Kk[1] - 1);
    PK[3] = -PK[3] * (Kk[1] - 1);
}


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速求平方根倒数
// 参数说明     num        输入数
// 返回参数     float      平方根倒数
// 使用示例     fast_invsqrt(5);
// 备注信息     函数内部调用
-------------------------------------------------------------------------------------------------------------------*/
float fast_invsqrt(float num)
{
    union
    {
            int i;
            float x;
    }u;
    u.x = num;
    u.i = 0x5f3759df - (u.i >> 1);
    u.x = u.x * (1.5f - (0.5f * num * u.x * u.x));
    u.x = u.x * (1.5f - (0.5f * num * u.x * u.x));
    return u.x;
}
