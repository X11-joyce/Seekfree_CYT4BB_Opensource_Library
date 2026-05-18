#ifndef CODE_MATRIX_H_
#define CODE_MATRIX_H_
#include "zf_common_headfile.h"



#define MAX_SIZE (4)

//变量

typedef float matrix_type;

typedef struct                  //矩阵结构体
{
    int rows;                  //行
    int cols;                  //列
    matrix_type data[MAX_SIZE][MAX_SIZE];
}matrix_t;

typedef struct
{
    matrix_type roll, pitch, yaw;
}EulerAngles;
extern EulerAngles euler_angle;

//函数
void Matrix_Init(matrix_t* martix, int rows, int cols);
void Matrix_Identity(matrix_t* matrix, int size);
void Matrix_From_Array(matrix_t* mat, const matrix_type* array,const int rows,const int cols);
matrix_t Matrix_Transpose(const matrix_t* src);
matrix_t multiply_matrices(const matrix_t* A, const matrix_t* B);
matrix_t add_matrices(const matrix_t* A, const matrix_t* B);
matrix_t subtract_matrices(const matrix_t* A, const matrix_t* B);
int inverse_matrix(matrix_t* A, matrix_t* invA);
static inline float invSqrt(float x);
void normalize_vector(matrix_t *v);
void print_matrix(const matrix_t* matrix);









#endif /* CODE_MATRIX_H_ */
