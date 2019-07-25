#ifndef SRC_MATH_MATRIX_H_
#define SRC_MATH_MATRIX_H_

#include <src/Util/Common/constants.h>

namespace Math
{
    class Matrix
    {
    public:
        double data[SYSTEM_ORDER][SYSTEM_ORDER];   // Values in matrix
    };

    class Vector
    {
    public:
        double data[SYSTEM_ORDER];   // Values in matrix
    };


    Matrix* matrix_product(Matrix *A, Matrix *B, Matrix *out);


    Vector* matrix_product(Matrix *A, Vector *B, Vector *out);


    Vector* matrix_product(Vector *A, Matrix *B, Vector *out);


    double matrix_product(Vector *A, Vector *B);


    Vector* matrix_product(Vector *A, double u, Vector *out);


    Matrix* matrix_sum(Matrix *A, Matrix *B, Matrix *out);


    Vector* matrix_sum(Vector *A, Vector *B, Vector *out);


    Matrix* matrix_diff(Matrix *A, Matrix *B, Matrix *out);


    Vector* matrix_diff(Vector *A, Vector *B, Vector *out);
};

#endif /* SRC_MATH_MATRIX_H_ */
