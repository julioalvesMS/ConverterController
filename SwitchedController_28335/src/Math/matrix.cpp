#include <src/Common/constants.h>

namespace Math
{
    class Matrix
    {
    public:
        double data[2][2];   // Values in matrix
    };

    class Vector
    {
    public:
        double data[2];   // Values in matrix
    };

    Matrix* matrix_product(Matrix *A, Matrix *B, Matrix *out)
    {
        int i, j, k;

        for (i=0; i<SYSTEM_ORDER; i++)
        {
            for (j=0; j<SYSTEM_ORDER; j++)
            {
                double sum = 0.0;
                for (k=0; k<SYSTEM_ORDER; k++)
                    sum += A->data[i][k]*B->data[k][j];

                out->data[i][j] = sum;
            }
        }

        return out;
    }

    Vector* matrix_product(Matrix *A, Vector *B, Vector *out)
    {
        int i, k;

        for (i=0;i<SYSTEM_ORDER;i++)
        {
            double sum = 0.0;
            for (k=0; k<SYSTEM_ORDER; k++)
                sum += A->data[i][k]*B->data[k];

            out->data[i] = sum;
        }

        return out;
    }

    Vector* matrix_product(Vector *A, Matrix *B, Vector *out)
    {
        int i, k;

        for (i=0;i<SYSTEM_ORDER;i++)
        {
            double sum = 0.0;
            for (k=0; k<SYSTEM_ORDER; k++)
                sum += A->data[k]*B->data[i][k];

            out->data[i] = sum;
        }

        return out;
    }

    double matrix_product(Vector *A, Vector *B)
    {
        double out = 0.0;
        int k;

        for (k=0; k<SYSTEM_ORDER; k++)
            out += A->data[k]*B->data[k];

        return out;
    }

    Vector* matrix_product(Vector *A, double u, Vector *out)
    {
        int i;

        for (i=0;i<SYSTEM_ORDER;i++)
        {
            out->data[i] = A->data[i] * u;
        }

        return out;
    }


    Matrix* matrix_sum(Matrix *A, Matrix *B, Matrix *out)
    {
        int i, j;

        for (i=0; i<SYSTEM_ORDER; i++)
        {
            for (j=0; j<SYSTEM_ORDER; j++)
            {
                out->data[i][j] = A->data[i][j] + B->data[i][j];
            }
        }

        return out;
    }


    Vector* matrix_sum(Vector *A, Vector *B, Vector *out)
    {
        int i;

        for (i=0; i<SYSTEM_ORDER; i++)
        {
            out->data[i] = A->data[i] + B->data[i];
        }

        return out;
    }


    Matrix* matrix_diff(Matrix *A, Matrix *B, Matrix *out)
    {
        int i, j;
        for (i=0;i<SYSTEM_ORDER;i++)
        {
            for (j=0;j<SYSTEM_ORDER;j++)
            {
                out->data[i][j] = A->data[i][j] - B->data[i][j];
            }
        }

        return out;
    }


    Vector* matrix_diff(Vector *A, Vector *B, Vector *out)
    {
        int i;

        for (i=0; i<SYSTEM_ORDER; i++)
        {
            out->data[i] = A->data[i] - B->data[i];
        }

        return out;
    }
};
