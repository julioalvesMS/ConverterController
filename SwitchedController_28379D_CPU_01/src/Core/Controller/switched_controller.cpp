#include <src/Core/Controller/switched_controller.h>

using namespace Math;

namespace Controller
{
    static Matrix P;

    Matrix* GetP()
    {
        //
        // Buck Converter - Rule 2
        //
        P.data[0][0] = 1.1656e-06;
        P.data[0][1] = 1.7630e-07;
        P.data[1][0] = 1.7630e-07;
        P.data[1][1] = 1.7635e-04;

        return &P;
    }
}
