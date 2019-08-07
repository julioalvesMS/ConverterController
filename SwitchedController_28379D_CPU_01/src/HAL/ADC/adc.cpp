#include <src/HAL/ADC/adc.h>

namespace ADC_HAL
{
    static double vout_mean_buffer[ADC_BUFFER_SIZE];
    static int buffer_index = 0;

    //
    // Configure - Write ADC configurations and power up the ADC for both
    //                ADC A and ADC B
    //
    void Configure(void)
    {
        int i;

        EALLOW;

        //
        //write configurations
        //
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4


        AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

        //
        //Set pulse positions to late
        //
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

        //
        //power up the ADC
        //
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

        //
        //delay for 1ms to allow ADC time to power up
        //
        DELAY_US(1000);

        EDIS;

        for(i=0;i<ADC_BUFFER_SIZE;i++)
        {
            vout_mean_buffer[i] = 0;
        }
    }

    void SetupADC(void)
    {
        SetupADC_A();
        SetupADC_B();
        SetupADC_C();
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_A(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_IL;     //SOC0 will convert the inductor current
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;              //sample window is 100 SYSCLK cycles
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SOURCE;  //trigger on ePWM1 SOCA/C
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              //end of SOC0 will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                //enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_B(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdcbRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_VIN;    //SOC0 will convert pin A0
        AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;              //sample window is 100 SYSCLK cycles
        AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SOURCE;  //trigger on ePWM1 SOCA/C
        AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              //end of SOC0 will set INT1 flag
        AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;                //enable INT1 flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_C(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdccRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdccRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_VOUT;   //SOC0 will convert pin A0
        AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;              //sample window is 100 SYSCLK cycles
        AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SOURCE;  //trigger on ePWM1 SOCA/C
        AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              //end of SOC0 will set INT1 flag
        AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;                //enable INT1 flag
        AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //make sure INT1 flag is cleared
        EDIS;

#if FILTERED_IL_SENSOR
        EALLOW;
        AdccRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_IL_AVG; //SOC0 will convert pin A0
        AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;              //sample window is 100 SYSCLK cycles
        AdccRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIG_SOURCE;  //trigger on ePWM1 SOCA/C
        EDIS;
#endif
    }

    //
    // ConfigureInterruption - Set the interruption handler and define where
    //                          to send the results from the conversion
    //
    void ConfigureInterruption(double* variables[], double* vout_mean_pointer)
    {
        int i;

        for (i=0;i<ADC_CONVERSIONS;i++)
        {
            resultDestination[i] = variables[i];
        }

        vout_mean = vout_mean_pointer;
    }


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt void Interruption(void)
    {
        //
        // Read ADC result
        //
        (*resultDestination[0]) = READ_IL(ADC_RESULT_IL); // Inductor Current
        (*resultDestination[1]) = READ_VOUT(ADC_RESULT_VOUT); // Load Voltage
        (*resultDestination[2]) = READ_VIN(ADC_RESULT_VIN); // Input Voltage


        DAC_HAL::Update(ADC_RESULT_IL, ADC_RESULT_VOUT);

        //
        // Mean Vout value
        //
        (*vout_mean) -= vout_mean_buffer[buffer_index];
        vout_mean_buffer[buffer_index] = (*resultDestination[1])/ADC_BUFFER_SIZE;
        (*vout_mean) += vout_mean_buffer[buffer_index];
        buffer_index = (buffer_index + 1) % ADC_BUFFER_SIZE;

        //
        // Prepare for next conversion
        //
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

        //
        // Check if overflow has occurred
        //
        if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }

        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
        return;
    }
}
