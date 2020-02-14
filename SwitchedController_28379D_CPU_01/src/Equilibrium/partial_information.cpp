#include <src/Equilibrium/partial_information.h>

namespace PartialInformation
{

    static double f[2] = {0, 0};
    static double i[2] = {0, 0};

    static double numLPF[2] = {0, 0};
    static double denLPF[2] = {0, 0};

    void Configure(void)
    {
        LoadFilter();
        ResetFilter();
    }

    void LoadFilter(void)
    {
        numLPF[0] = 0.0023;
        numLPF[1] = 0.0023;

        denLPF[0] = 1;
        denLPF[1] = -0.9954;
    }

    void UpdateReference(double IL)
    {
        i[1] = i[0];
        f[1] = f[0];

        i[0] = IL;
        f[0] = numLPF[0]*i[0] + numLPF[1]*i[1] - denLPF[1]*f[1];

        Xe[0] = f[0];
        Xe[1] = Vref;
    }

    void ResetFilter(void)
    {
        f[0] = 0;
        f[1] = 0;

        i[0] = 0;
        i[1] = 0;
    }
}
