#include "Source/SwitchedSystem/switched_system.h"


static Matrix *csi;


static Matrix *prod_01, *prod_02; // Ai*xe, Bi*u
static Matrix *sum_0;   // Ai*xe + Bi*u

static Matrix *prod_1;  // csi'*P
static Matrix *prod_2;  // csi'*P * (Ai*xe + Bi*u)


int rule2_init(SwitchedSystem *sys)
{
    int n = sys->subSystems[0].A->n;

    // Create all vectors
    csi = matrix_create(n, 1);

    prod_01 = matrix_create(n, 1);
    prod_02 = matrix_create(n, 1);
    sum_0 = matrix_create(n, 1);
    prod_1 = matrix_create(1, n);
    prod_2 = matrix_create(1, 1);
}


int rule2_switchingRule(SwitchedSystem *sys, Matrix *P, Matrix *X, Matrix *Xe)
{
    double best_sigma = 0, new_sigma;
    int best_id = 0;
    int i;

    matrix_diff(X, Xe, csi);

    for (i=0;i<sys->N;i++){



        if (new_sigma < best_sigma)
            best_id = i;
    }

    return best_id;
}

double rule2_evaluateSubSystem(SubSystem *subSys, Matrix *P, Matrix *X, Matrix *Xe, double u)
{
    matrix_product(subSys->A, Xe, prod_01);
    matrix_product(subSys->B, Xe, prod_02);

    return 0;
}
