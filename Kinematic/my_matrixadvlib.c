/*===========================================================================
 *MatrixLib modified by MrWang_tju  2.19.4.22
 *convert pointer to struct for avoiding use the function
  malloc() and free()

 *=========================================================================*/
#include "my_matrixadvlib.h"


/*===========================================================================
 * LUdecomposition
 * Given a matrix A and two more null values, the function uses Crout's
 * algorithm (page 44 of Numerical Recipes in C) to perform the
 * LU decomposition of matrix A.
 *=========================================================================*/
void m_LUdecomposition(Matrix_t a, Matrix_t * l, Matrix_t * u) {
    int i, j, k;
//     double* ptrA;
     double* ptrL;
//     double* ptrU;
     double sum;

    assert(a.width == a.height, "Matrix A must be square");

    //确保 L U 矩阵为 null
    for(i=0;i<(a.width*a.height);i++)
    {
        (*l).data[i] = 0;
        (*u).data[i] = 0;
    }

    // assert(l == NULL && u == NULL, "Matricies L and U must be null");

    // Step 1: Assign 1 to the diagonal of the lower matrix.
    ptrL = (l)->data;
    for (i = 0; i < a.width; i++) {
        *ptrL = 1.0;
        ptrL += a.width + 1;
    }

    // Step 2
    for (j = 0; j < a.width; j++) {

        // Part A: Solve for the upper matrix.
        for (i = 0; i <= j; i++) {

            sum = 0.0;
            for (k = 0; k < i; k++) {
                sum += (*l).data[i * a.width + k] * (*u).data[k * a.width + j];
            }

            (*u).data[i * a.width + j] = a.data[i * a.width + j] - sum;
        }

        // Part B: Solve fpr the lower matrix
        for (i = j+1; i < a.width; i++) {

            sum = 0.0;
            for (k = 0; k < j; k++) {
                sum += (*l).data[i * a.width + k] * (*u).data[k * a.width + j];
            }

            (*l).data[i * a.width + j] = 1.0/(*u).data[j * a.width + j] * ( a.data[i * a.width + j] - sum);
        }
    }

    return;
}

void m_LUdecomposition_3x3(Matrix_t_3x3 a, Matrix_t_3x3 * l, Matrix_t_3x3 * u) {
    int i, j, k;
    double* ptrA;
    double* ptrL;
//    double* ptrU;
    double sum;

    assert(a.width == a.height, "Matrix A must be square");

    //确保 L U 矩阵为 null
    for(i=0;i<(a.width*a.height);i++)
    {
            (*l).data[i] = 0;
            (*u).data[i] = 0;
    }

    // assert(l == NULL && u == NULL, "Matricies L and U must be null");

    // Step 1: Assign 1 to the diagonal of the lower matrix.
    ptrL = (l)->data;
    for (i = 0; i < a.width; i++) {
        *ptrL = 1.0;
        ptrL += a.width + 1;
    }

    // Step 2
    for (j = 0; j < a.width; j++) {

        // Part A: Solve for the upper matrix.
        for (i = 0; i <= j; i++) {

            sum = 0.0;
            for (k = 0; k < i; k++) {
                sum += (*l).data[i * a.width + k] * (*u).data[k * a.width + j];
            }

            (*u).data[i * a.width + j] = a.data[i * a.width + j] - sum;
        }

        // Part B: Solve fpr the lower matrix
        for (i = j+1; i < a.width; i++) {

            sum = 0.0;
            for (k = 0; k < j; k++) {
                sum += (*l).data[i * a.width + k] * (*u).data[k * a.width + j];
            }

            (*l).data[i * a.width + j] = 1.0/(*u).data[j * a.width + j] * ( a.data[i * a.width + j] - sum);
        }
    }

    return;
}


/*===========================================================================
 * solver
 * Solves a system of liner equations using LU decomposition. This requires
 * an A: n by n matrix and B: n by p matrix, where
 * A * X = b
 * The algorithm returns X, which will be a n by p matrix.
 *
 * This algorithm is described on page 121 of the book "Matrix Computations"
 * by Golub and Loan.
 *=========================================================================*/
Matrix_t m_solver(Matrix_t a, Matrix_t b) {
    int i, j, k;
    double sum;
    Matrix_t l;
    Matrix_t u;
    Matrix_t y;
    Matrix_t x;
    double* row;

    assert(a.height == b.height, "In the solver, both matrices should have the same height.");

    m_LUdecomposition(a, &l, &u);

    // printf("L: \n");
    // printDAta(l.data,36);

    // printf("U: \n");
    // printDAta(u.data,36);

    // y = makeMatrix(1, a.height);
    // x = makeMatrix(b.width, b.height);

    for (k = 0; k < b.width; k++) {
        // Perform backward subsitituion with L
        // L * y = B_k
        for (i = 0; i < a.height; i++) {
            row = l.data + i * a.width;
            sum = 0;
            for (j = 0; j < i; j++) {
                sum += y.data[j] * (*row++);
            }

            y.data[i] = (b.data[i * b.width + k] - sum) / *row;
        }

        // Perform backward subsitituion again with U
        // U * x = y
        for (i = a.height - 1; i >= 0; i--) {
            row = u.data + i * a.width + (a.width - 1);
            sum = 0;
            for (j = a.width - 1; j > i; j--) {
                sum += x.data[j * b.width + k] * (*row--);
            }

            x.data[i * b.width + k] = (y.data[i] - sum) / *row;
        }
    }

    // freeMatrix(l);
    // freeMatrix(u);
    // freeMatrix(y);
    return x;
}


Matrix_t_3x1 m_solver_3x3(Matrix_t_3x3 a, Matrix_t_3x1 b) {
    int i, j, k;
    double sum;
    Matrix_t_3x3 l;
    Matrix_t_3x3 u;
    Matrix_t_3x3 y;
    Matrix_t_3x1 x;
    double* row;

    assert(a.height == b.height, "In the solver, both matrices should have the same height.");

    m_LUdecomposition_3x3(a, &l, &u);

    // printf("L: \n");
    // printDAta(l.data,36);

    // printf("U: \n");
    // printDAta(u.data,36);

    // y = makeMatrix(1, a.height);
    // x = makeMatrix(b.width, b.height);

    for (k = 0; k < b.width; k++) {
        // Perform backward subsitituion with L
        // L * y = B_k
        for (i = 0; i < a.height; i++) {
            row = l.data + i * a.width;
            sum = 0;
            for (j = 0; j < i; j++) {
                sum += y.data[j] * (*row++);
            }

            y.data[i] = (b.data[i * b.width + k] - sum) / *row;
        }

        // Perform backward subsitituion again with U
        // U * x = y
        for (i = a.height - 1; i >= 0; i--) {
            row = u.data + i * a.width + (a.width - 1);
            sum = 0;
            for (j = a.width - 1; j > i; j--) {
                sum += x.data[j * b.width + k] * (*row--);
            }

            x.data[i * b.width + k] = (y.data[i] - sum) / *row;
        }
    }

    // freeMatrix(l);
    // freeMatrix(u);
    // freeMatrix(y);
    return x;
}


/*===========================================================================
 * inverseMatrix
 * Given a matrix A, returns the determinant.
 *
 * This algorithm is described on page 121 of the book "Matrix Computations"
 * by Golub and Loan.
 *=========================================================================*/
Matrix_t m_inverseMatrix(Matrix_t a) {
    Matrix_t eye;
    Matrix_t inv;

    assert(a.width == a.height, "Matrix A must be square.");
    eye = m_eyeMatrix(a.width);
//    printf("eye Matrix!\n");
//	printDAta(eye.data,36);

    a.width = a.height = 6;
    eye.height = eye.width = 6;

    // printDAta(a.data,36);
    inv = m_solver(a, eye);
    
//    printf("%6.3f \n",inv.data[1]);
//	printf("Inv Matrix\n");
//    printDAta(inv.data,36); //必须加printf输出 inv的值 不加 数据传递失败  可能原因是 不输出 在变量生命周期内 变量没有被使用 而被编译器优化
   
    return inv;
}

/*===========================================================================
 * eyeMatrix
 * Returns an identity matrix of size n by n, where n is the input
 * parameter.
 *=========================================================================*/
Matrix_t m_eyeMatrix(int n) {
    int i;
    Matrix_t out;
     double* ptr;

    assert(n > 0, "Identity matrix must have value greater than zero.");
    // out = makeMatrix(n, n);
    ptr = out.data;
    for(i=0;i<n*n;i++)
    {
        *(ptr+i) = 0;
    }
    for (i = 0; i < n; i++) {
        *ptr = 1.0;
        ptr += n + 1;
    }
    return out;
}

void printDAta(double *data,int len)
{
    int i;
    for(i=0;i<len;i++)
    {
        if(i%6==0)
            printf("\n\r");
        printf("%9.6f  ",*(data+i));
    }
}

