#include <src/Sensor/sensor.h>

namespace Sensor
{
    //
    // VALOR MÉDIO DA TENSÃO DE SAÍDA
    //
    static double vout_mean_buffer[ADC_BUFFER_SIZE];
    static int buffer_index = 0;

    //
    // VARIÁVEIS COM OS VALORES DOS SENSORES
    //
    static double s_state[SYSTEM_ORDER];    // X - State vector
    static double input_voltage = 0;        // u - Input Voltage
    static double output_current = 0;       // Iout - Output Current
    static double vout_mean = 0;            // Vout - Output mean value

    void Configure(void)
    {
        int i;

        //
        // Configuração do canais analógicos de entrada
        //
        Setup_ADC();        // Configura canais de de ADC e interrupções para leitura

        //
        // Configuração do canais de PWM e TZ
        //
        Setup_ePWM();

        //
        // Assume the initial state as zero
        //
        s_state[0] = 0;
        s_state[1] = 0;

        vout_mean = 0;
        input_voltage = 0;
        output_current = 0;

        for(i=0;i<ADC_BUFFER_SIZE;i++)
            vout_mean_buffer[i] = 0;

    }

    void ConfigureFrequency(int pwmTBPRD)
    {
        EPwm1Regs.TBPRD = pwmTBPRD;     // 125 = 200 kHz
        EPwm1Regs.TBCTR = 0x0000;       // Clear counter
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

    //
    //  GetOutput - Get the pointer to the variable where the input
    //              voltage is stored
    //
    double* GetOutputCurrent(void)
    {
        return &output_current;
    }

    void ReadADCResult(void)
    {
        //
        // Read ADC result
        //
        s_state[0] = READ_IL(ADC_RESULT_IL);        // Current
        s_state[1] = READ_VOUT(ADC_RESULT_VOUT);    // Voltage
        input_voltage = READ_VIN(ADC_RESULT_VIN);   // Input Voltage
        output_current = READ_IOUT(ADC_RESULT_IOUT);   // Output Current

        vout_mean -= vout_mean_buffer[buffer_index];
        vout_mean_buffer[buffer_index] = (s_state[1])/ADC_BUFFER_SIZE;
        vout_mean += vout_mean_buffer[buffer_index];
        buffer_index = (buffer_index + 1) % ADC_BUFFER_SIZE;

        return;
    }
}
