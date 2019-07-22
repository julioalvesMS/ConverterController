#include <src/Core/Sensor/sensor.h>

extern void InitAdc(void);

using namespace Math;

namespace Sensor
{
    static Vector s_state;

    static double input_voltage = 0;

    double raw_variables[ADC_CONVERSIONS];

    //
    // Configure - Make configurations and initializations needed
    //              to read the sensors
    //
    void Configure(void)
    {
        ADC::Configure();

        PWM::Configure();

        ADC::SetupADC();

        //
        // Configure the interruption to read the ADC
        //
        double* variables[ADC_CONVERSIONS] = {
            &(raw_variables[0]),
            &(raw_variables[1]),
            &(raw_variables[2])
        };
        ADC::ConfigureInterruption(variables);

        //
        // Assume the initial state as zero
        //
        s_state.data[0] = 0;
        s_state.data[1] = 0;
    }

    //
    //  GetState - Get the pointer to the variable where the state
    //              vector is stored
    //
    Vector* GetState(void)
    {
        return &s_state;
    }

    //
    //  GetState - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetInput(void)
    {
        return &input_voltage;
    }

    //
    //  UpdateState - Update the state vector with the last read
    //
    void UpdateState(void)
    {
        s_state.data[0] = raw_variables[0];
        s_state.data[1] = raw_variables[1];
    }

    //
    //  GetState - Update the input voltage with the last read
    //
    void UpdateInput(void)
    {
        input_voltage = raw_variables[2];
    }
}
