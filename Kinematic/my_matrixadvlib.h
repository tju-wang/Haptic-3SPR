#ifndef _MY_MATRIXADVLIB_H
#define _MY_MATRIXADVLIB_H
#include "matrixlib.h"
#include <stddef.h>
#include "stdio.h"

// void LUdecomposition(matrix* a, matrix** l, matrix** u);
void m_LUdecomposition(Matrix_t a, Matrix_t* l, Matrix_t* u);
// double determinantMatrix(matrix* a);
// matrix* inverseMatrix(matrix* a);
// Matrix_t m_inverseMatrix(Matrix_t a);
Matrix_t m_inverseMatrix(Matrix_t a);
Matrix_t m_solver(Matrix_t a, Matrix_t b);
// Matrix_t m_solver(Matrix_t a, Matrix_t b,Matrix_t *inv);

Matrix_t m_eyeMatrix(int n);

void printDAta(double *data,int len);
Matrix_t_3x1 m_solver_3x3(Matrix_t_3x3 a, Matrix_t_3x1 b);
void m_LUdecomposition_3x3(Matrix_t_3x3 a, Matrix_t_3x3 * l, Matrix_t_3x3 * u);

#endif

