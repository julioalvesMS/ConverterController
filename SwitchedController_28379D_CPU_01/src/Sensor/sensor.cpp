#include <src/Sensor/sensor.h>

namespace Sensor
{
    //
    // VARIÁVEIS COM OS VALORES DOS SENSORES
    //
    static double s_state[SYSTEM_ORDER];    // X - State vector
    static double input_voltage = 0;        // u - Input Voltage
    static double output_current = 0;       // Iout - Output Current
    static double vout_mean = 0;            // Vout - Output mean value

    void Configure(void)
    {
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

    float filtro_1(float vP)
    {
        static float Bv[4]={0.000027213807988318872, 2*0.0000272138079883188725,0.000027213807988318872  };//50Hz em 30k
        static float Av[4]={1,-1.985190657896261,0.9852995131282144 };

        static float xv[4] = {0,0,0,0};
        static float yv[4] = {0,0,0,0};


        xv[0] = 1*vP; // adc voltage value is the current sample

        // calculate filter value
        yv[0] = Bv[0]*xv[0] + Bv[1]*xv[1] + Bv[2]*xv[2] - Av[1]*yv[1] - Av[2]*yv[2] ;//- A[3]*y[3];+ B[3]*x[3]


        // save past states so ready for next sample

        //x[3] = x[2];
        xv[2] = xv[1];
        xv[1] = xv[0];

        //y[3] = y[2];
        yv[2] = yv[1];
        yv[1] = yv[0];


        return yv[0];
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

        vout_mean = filtro_1(s_state[1]);

        return;
    }
}
