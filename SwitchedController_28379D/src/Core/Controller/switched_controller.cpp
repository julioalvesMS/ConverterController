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
        P.data[0][0] = 3.6283e-04;
        P.data[0][1] = 2.2042e-05;
        P.data[1][0] = 2.2042e-05;
        P.data[1][1] = 1.6484e-04;

        return &P;
    }
}
