#include <src/Controller/ClassicController/state_feedback_h2_controller.h>

namespace StateFeedbackH2Controller
{
    static double K[2] = {0, 0};
    static double M = 0;

    void LoadController(void)
    {
        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            Buck::GetStateFeedbackH2Controller(K, &M);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetStateFeedbackH2Controller(K, &M);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetStateFeedbackH2Controller(K, &M);
            break;

        default:
            break;
        }
    }


    double Update(double Xe[SYSTEM_ORDER], double X[SYSTEM_ORDER])
    {
        double r, kx;
        double upperLimit, lowerLimit;
        double duty;

        lowerLimit = -0.1;

        switch (activeConverter)
        {
        case BaseConverter::ID_Boost:
            upperLimit = 0.9*(1 - sqrt(Rratio));
            break;

        case BaseConverter::ID_BuckBoost:
            upperLimit = 0.9*(Rratio + 1 - sqrt(Rratio*(1 + Rratio)));
            break;

        default:
            upperLimit = 1;
            break;
        }

        r = Xe[1];
        kx = K[0]*X[0] + K[1]*X[1];

        duty = M*r - kx;

        if (duty > upperLimit) duty = upperLimit;
        else if (duty < lowerLimit) duty = lowerLimit;

        if (duty > 1) duty = 1;
        else if (duty < 0) duty = 0;

        return duty;
    }
}
