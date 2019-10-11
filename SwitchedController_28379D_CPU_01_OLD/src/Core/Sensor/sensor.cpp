#include <src/Core/Sensor/sensor.h>

namespace Sensor
{
    static double s_state[SYSTEM_ORDER];

    static double input_voltage = 0;

    static double vout_mean = 0;

    //
    // Configure - Make configurations and initializations needed
    //              to read the sensors
    //
    void Configure(void)
    {
        ADC_HAL::Configure();

        PWM::Configure();

        ADC_HAL::SetupADC();

        //
        // Configure the interruption to read the ADC
        //
        double* variables[ADC_CONVERSIONS] = {
            &(s_state[0]),
            &(s_state[1]),
            &(input_voltage)
        };
        ADC_HAL::ConfigureInterruption(variables, &vout_mean);

        //
        // Assume the initial state as zero
        //
        s_state[0] = 0;
        s_state[1] = 0;
    }

    //
    //  Start - Start Sensor aquisition
    //
    void Start(void)
    {
        PWM::Start();
    }

    //
    //  GetState - Get the pointer to the variable where the state
    //              vector is stored
    //
    double* GetState(void)
    {
        return s_state;
    }

    //
    //  GetInput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void)
    {
        return &input_voltage;
    }

    //
    //  GetOutput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetOutput(void)
    {
        return &vout_mean;
    }
}
