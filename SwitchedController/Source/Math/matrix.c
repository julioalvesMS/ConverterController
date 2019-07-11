#include <stdlib.h>

#include "matrix.h"

Matrix* matrix_create(int m, int n){
    Matrix *A;
    int i;

    A = (Matrix*) malloc(sizeof(Matrix));

    A->m = m;
    A->n = n;

    A->data = (double**) malloc(sizeof(double*)*A->m);

    for (i=0;i<A->m;i++)
    {
        A->data[i] = (double*) malloc(sizeof(double)*A->n);
    }

    return A;
}


void matrix_free(Matrix *A){
    int i;

    for (i=0;i<A->m;i++)
    {
        free(A->data[i]);
    }

    free(A->data);

    free(A);
}

void matrix_product(Matrix *A, Matrix *B, Matrix *out){
    int i, j, k;
    for (i=0;i<out->m;i++)
    {
        for (j=0;j<out->n;j++)
        {
            double sum = 0.0;
            for (k=0;k<A->n;k++)
                sum += A->data[i][k]*B->data[k][j];

            out->data[i][j] = sum;
        }
    }
}


void matrix_sum(Matrix *A, Matrix *B, Matrix *out){
    int i, j;
    for (i=0;i<out->m;i++)
    {
        for (j=0;j<out->n;j++)
        {
            out->data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }
}


void matrix_diff(Matrix *A, Matrix *B, Matrix *out){
    int i, j;
    for (i=0;i<out->m;i++)
    {
        for (j=0;j<out->n;j++)
        {
            out->data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }
}
