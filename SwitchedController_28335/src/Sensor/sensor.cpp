#include <src/Sensor/sensor.h>

namespace Sensor
{
    void Configure(void)
    {
        //
        // Configuração do canais analógicos de entrada
        //
        InitAdc();          // Inicializa periférico ADC
        Setup_ADC();        // Configura canais de de ADC e interrupções para leitura

        //
        // Configuração do canais de PWM e TZ
        //
        Setup_ePWM();

        //
        // CONFIGURAÇÃO DA DAC POR SPI
        //
        spi_fifo_init();    // Configura SPI-FIFO DA DAC
        spi_init();         // Inicializa SPI-FIFO DA DAC

        //
        // Assume the initial state as zero
        //
        s_state[0] = 0;
        s_state[1] = 0;

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

    __interrupt
    void Interruption(void)
    {
        // Test GPIO. Used to enable frequency measurement
        GpioDataRegs.GPATOGGLE.bit.AF = 1;

        //
        // Read ADC result
        //
        s_state[0] = READ_IL(ADC_RESULT_IL);        // Current
        s_state[1] = READ_VOUT(ADC_RESULT_VOUT);    // Voltage
        input_voltage = READ_VIN(ADC_RESULT_VIN);   // Input Voltage

        vout_mean -= vout_mean_buffer[buffer_index];
        vout_mean_buffer[buffer_index] = (s_state[1])/ADC_BUFFER_SIZE;
        vout_mean += vout_mean_buffer[buffer_index];
        buffer_index = (buffer_index + 1) % ADC_BUFFER_SIZE;

        //
        // Reinitialize for next ADC sequence
        //
        AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

        return;
    }
}
