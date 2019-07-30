#include <src/Core/Sensor/sensor.h>

extern void InitAdc(void);

namespace Sensor
{
    static double s_state[2];

    static double input_voltage = 0;

    //
    // Configure - Make configurations and initializations needed
    //              to read the sensors
    //
    void Configure(void)
    {
        ADC_HAL::Configure();

        PWM::ADC_Configure();

        ADC_HAL::SetupADC();

        //
        // Configure the interruption to read the ADC
        //
        double* variables[ADC_CONVERSIONS] = {
            &(s_state[0]),
            &(s_state[1]),
            &(input_voltage)
        };
        ADC_HAL::ConfigureInterruption(variables);

        //
        // Assume the initial state as zero
        //
        s_state[0] = 0;
        s_state[1] = 0;
    }

    void Start(void)
    {
        PWM::ADC_Start();
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
    //  GetState - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void)
    {
        return &input_voltage;
    }
}
