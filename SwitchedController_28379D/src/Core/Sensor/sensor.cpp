#include <src/Core/Sensor/sensor.h>

extern void InitAdc(void);

using namespace Math;

namespace Sensor
{
    static Vector s_state;

    static double input_voltage = 0;

    static unsigned int raw_variables[ADC_CONVERSIONS];

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
        unsigned int* variables[ADC_CONVERSIONS] = {
            &(raw_variables[0]),
            &(raw_variables[1]),
            &(raw_variables[2])
        };
        ADC_HAL::ConfigureInterruption(variables);

        //
        // Assume the initial state as zero
        //
        s_state.data[0] = 0;
        s_state.data[1] = 0;
    }

    void Start(void)
    {
        PWM::Start();
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
        s_state.data[0] = ADCDRV_1ch_F_C(raw_variables[0]);
        s_state.data[1] = ADCDRV_1ch_F_C(raw_variables[1]);
    }

    //
    //  GetState - Update the input voltage with the last read
    //
    void UpdateInput(void)
    {
        input_voltage = ADCDRV_1ch_F_C(raw_variables[2]);
    }
}
