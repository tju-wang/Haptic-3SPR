#ifndef _MATRIXLIB_H
#define _MATRIXLIB_H

/*
 * This is my personal matrix C library.
 * Now I understand why the NR library was created.
 * I didn't realize how quickly I would run into
 * memory problems when working with matricies.
 *
 * None of the matrix operations below change the
 * input matrices if an input is required. They all
 * return a new matrix with the new changes.
 *
 * Because memory issues are so prevelant, be sure
 * to use the "freeMatrix" function to reclaim some
 * of that memory.
 *
 * Most of the algorithms come from the following books:
 *
 * Numerical Recipes in C (2nd Edition)
 * 1992
 * By Press, Teukolsky, Vetterling, and Flannery
 *
 * Matric Computations (3rd Edition)
 * 1996
 * By Golub and Loan
 */

typedef struct _matrix {
    int height;
    int width;
    double* data;
} matrix;

typedef struct _Matrix {
    int height;
    int width;
    double data[36];
} Matrix_t;

typedef struct _Matrix_3x3 {
    int height;
    int width;
    double data[9];
} Matrix_t_3x3;

typedef struct _Matrix_3x1 {
    int height;
    int width;
    double data[3];
} Matrix_t_3x1;


//============================
// Stuff I wish C had.
//============================
void assert(int assertion, char* message);

//============================
// Catch and release functions
//============================
matrix* readMatrix(char* filename);
matrix* makeMatrix(int width, int height);
matrix* copyMatrix(matrix* m);
void freeMatrix(matrix* m);
void writeMatrix(matrix* m, char* filename);
void printMatrix(matrix* m);

//============================
// Basic Matrix operations
//============================
matrix* eyeMatrix(int n);
// Matrix_t eyeMatrix(int n);
double traceMatrix(matrix* m);
matrix* transposeMatrix(matrix* m);
matrix* meanMatrix(matrix* m);
matrix* multiplyMatrix(matrix* a, matrix* b);
matrix* scaleMatrix(matrix* m, double value);
matrix* covarianceMatrix(matrix* m);
void rowSwap(matrix* a, int p, int q); // This method changes the input matrix.
matrix* dotProductMatrix(matrix* a, matrix* b);
matrix* dotDiagonalMatrix(matrix* a, matrix* b);
matrix* L2_distance(matrix* a, matrix* b);

#endif


