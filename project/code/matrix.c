#include "zf_common_headfile.h"
#include "matrix.h"
/*****
* Function: Matrix_Init

* Description: 初始化矩阵并进行清0赋值

* Parameters:
 *   - martix: 指向要初始化的矩阵结构体的指针
 *   - rows:   矩阵的行数（必须为正整数）
 *   - cols:   矩阵的列数（必须为正整数）

****/

void Matrix_Init(matrix_t* martix, int rows, int cols)
{
    martix->rows = rows;
    martix->cols = cols;
    memset(martix->data, 0, MAX_SIZE * MAX_SIZE * sizeof(matrix_type));
}


/*****
 * Function：Matrix_Identity

 * Description: 初始化一个单位矩阵（对角线元素为1，其余为0）

 * Parameters:
  * - martix: 指向要初始化的矩阵结构体的指针
  * - size ：矩阵的维度

 *****/

void Matrix_Identity(matrix_t* matrix, int size)
{
    matrix->rows = size;
    matrix->cols = size;
    memset(matrix->data, 0, sizeof(matrix->data));
    for(int i = 0; i < size; i++)
    {
        matrix->data[i][i] = 1.0f;
    }
}


/*****
 * Function：Matrix_From_Array

 * Description：将一维数组数据填充到矩阵结构中

 * Parameters：
  * - mat:    输出参数矩阵
  * - array:  输入数列指针
  * - rows:   矩阵行数
  * - cols:   矩阵列数

 *****/

void Matrix_From_Array(matrix_t* mat, const matrix_type* array,const int rows,const int cols)
{
    Matrix_Init(mat, rows, cols);
    for(int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            mat->data[i][j] = array[i * cols + j];
        }
    }
}

/*****
 * Function：Matrix_Transpose

 * Description：生成输入矩阵的转置矩阵

 * Parameters：
  * - src: 数入矩阵指针

 * return ：
  * - 新的转置矩阵

 *****/
matrix_t Matrix_Transpose(const matrix_t* src)
{
    // 设置目标矩阵的大小
    matrix_t dest;
    Matrix_Init(&dest, src->cols, src->rows);
    // 进行转置操作
    for(int i = 0; i < src->rows; i++)
    {
        for(int j = 0; j < src->cols; j++)
        {
            dest.data[j][i] = src->data[i][j];
        }
    }
    return dest;
}

/*****
 * Function：multiply_matrices

 * Description：计算两个矩阵的乘积（A × B）

 * Parameters：
 * - A: 左矩阵指针（m×n阶）
 * - B: 右矩阵指针（n×p阶）

 * return：
  * - 返回乘积矩阵（m×p阶）

 *****/

matrix_t multiply_matrices(const matrix_t* A, const matrix_t* B)
{
    // 初始化结果矩阵
    matrix_t dest;
    Matrix_Init(&dest, A->rows, B->cols);

    // 进行矩阵乘法运算
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < B->cols; j++)
        {
            for(int k = 0; k < A->cols; k++)
            {
                dest.data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }

    return dest;
}

/*****
 * Function：add_matrices

 * Description：计算两个矩阵的加法（A + B）

 * Parameters：
  * - A: 第一个矩阵指针（m×n阶）
  * - B: 第二个矩阵指针（m×n阶）

 * return：
  * - 返回和矩阵（m×n阶）

 *****/
matrix_t add_matrices(const matrix_t* A, const matrix_t* B)
{
    matrix_t result;
    Matrix_Init(&result, A->rows, A->cols);

    // 进行矩阵加法运算
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }

    return result;  // 返回加法结果矩阵
}

/*****
 * Function：subtract_matrices

 * Description：计算两个矩阵的减法（A - B）

 * Parameters：
  * - A: 被减矩阵指针（m×n阶）
  * - B: 减矩阵指针（m×n阶）

 * return：
  * - 返回差矩阵（m×n阶）

 *****/
matrix_t subtract_matrices(const matrix_t* A, const matrix_t* B)
{

    // 初始化结果矩阵
    matrix_t result;
    Matrix_Init(&result, A->rows, A->cols);

    // 进行矩阵减法运算
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }

    return result;  // 返回减法结果矩阵
}

/*****
 * Function：inverse_matrix

 * Description：使用高斯-约旦消元法计算方阵的逆矩阵

 * Parameters：
  * - A:    输入矩阵指针（n×n阶）
  * - invA: 逆矩阵输出指针（n×n阶）

 * return：
  * - 0: 成功计算出逆矩阵
  * - 1: 矩阵不可逆（奇异矩阵）

 *****/
int inverse_matrix(matrix_t* A, matrix_t* invA)
{

    const matrix_type THRESHOLD = 1e-6;  // 根据具体情况调整阈值

    Matrix_Init(invA, A->rows, A->cols);

    int n = A->rows;

    matrix_type augmented[MAX_SIZE][2 * MAX_SIZE];

    // 构造增广矩阵 [A | I]
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            augmented[i][j] = A->data[i][j];
            augmented[i][j + n] = (float)((i == j) ? 1 : 0);  // 设置单位矩阵部分
        }
    }

    // 高斯消元过程
    for(int i = 0; i < n; i++)
    {
        // 找到第 i 列的主元素
        int max_row = i;
        for(int j = i + 1; j < n; j++)
        {
            if(fabs(augmented[j][i]) > fabs(augmented[max_row][i]))
            {
                max_row = j;
            }
        }

        // 如果主元素为 0，说明矩阵不可逆
        if(fabs(augmented[max_row][i]) < THRESHOLD)
        {
            return 1;  // 不可逆
        }

        // 交换当前行和最大行
        if(max_row != i)
        {
            for(int j = 0; j < 2 * n; j++)
            {
                matrix_type temp = augmented[i][j];
                augmented[i][j] = augmented[max_row][j];
                augmented[max_row][j] = temp;
            }
        }

        // 对当前行进行归一化，使得主元素为 1
        matrix_type pivot = augmented[i][i];
        for(int j = 0; j < 2 * n; j++)
        {
            augmented[i][j] /= pivot;
        }

        // 消去当前列其他行的元素
        for(int j = 0; j < n; j++)
        {
            if(j != i)
            {
                matrix_type factor = augmented[j][i];
                for(int k = 0; k < 2 * n; k++)
                {
                    augmented[j][k] -= factor * augmented[i][k];
                }
            }
        }
    }

    // 提取逆矩阵部分 [I | A^-1]
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            invA->data[i][j] = augmented[i][j + n];
        }
    }

    return 0;  // 可逆，返回 0
}
/*****
* Function：inline float

* Description：快速计算平方根倒数

* Parameters：
 * - X :输入量


*****/


static inline float invSqrt(float x)
{
    float xhalf = 0.5f * x;

    int i = *(int*)&x;

    i = 0x5f375a86 - (i >> 1);

    x = *(float*)&i;

    x = x * (1.5f - xhalf * x * x);

    return x;
}

/*****
* Function：normalize_vector

* Description：向量归一化函数

* Parameters：
 * -V :数入矩阵


*****/


void normalize_vector(matrix_t *v)
{
    matrix_type norm = 0;

    if(1 == v->rows)
    {
        for(int i = 0; i < v->cols; ++i)
        {
            norm += (v->data[0][i] * v->data[0][i]);
        }
    }
    if(1 == v->cols)
    {
        for(int i = 0; i < v->rows; ++i)
        {
            norm += (v->data[i][0] * v->data[i][0]);
        }
    }

    norm = invSqrt((float)norm);
    if(1 == v->rows)
    {
        for(int i = 0; i < v->cols; ++i)
        {
            v->data[0][i] *= norm;
        }
    }
    if(1 == v->cols)
    {
        for(int i = 0; i < v->rows; ++i)
        {
            v->data[i][0] *= norm;
        }
    }
}



/*****
* Function：print_matrix

* Description：打印矩阵

* Parameters：数入矩阵
 ****/

void print_matrix(const matrix_t* matrix)
{
    for(int i = 0; i < matrix->rows; i++)
    {
        for(int j = 0; j < matrix->cols; j++)
        {
            printf("%2f ", matrix->data[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}
