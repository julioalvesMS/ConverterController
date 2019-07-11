#ifndef _matrix_h_
#define _matrix_h_


typedef struct matrix {
   int m;   // Lines
   int n;   // Columns

   double **data;   // Values in matrix

} Matrix;


Matrix* matrix_create(int m, int n);


void matrix_free(Matrix *A);


void matrix_product(Matrix *A, Matrix *B, Matrix *out);


void matrix_sum(Matrix *A, Matrix *B, Matrix *out);


void matrix_diff(Matrix *A, Matrix *B, Matrix *out);

#endif
