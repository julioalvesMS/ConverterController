#include <src/Core/Controller/pid.h>

namespace PID
{
    double Update(double Vref, double Vout, double Vin)
    {
        double erro = Vref - Vout;

        pid_sum += erro*(ADC_PERIOD)/PERIOD_UNIT;

        double DutyCycle = pid_kp*erro + pid_ki*pid_sum;

        return DutyCycle;
    }
}
