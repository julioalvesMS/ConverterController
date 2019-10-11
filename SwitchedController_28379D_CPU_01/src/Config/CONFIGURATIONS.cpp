#include <src/Config/CONFIGURATIONS.h>     // Device Headerfile and Examples Include File


//################## ROTINAS CONFIGURAÇÃO DO FUNCIONAMENTO DOS PERIFÉRICOS#############################



#define ADC_TRIG_SOURCE         5   // Triggered by PWM1A

//############CONFIGURAÇÃO DOS CANAIS ANALÓGICOS ADC ###################

void Setup_ADC(void)
{

    Uint16 acqps;

EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);
EDIS;

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

EALLOW;     // ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 0;  // Canal A0 - Vy
    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = 2;  // Canal A2 - Vx
    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = 3;  // Canal A3 - Iy
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 0;  // Canal A0 - Vy
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = 1;  // Canal A1 - Iv
//    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = 2;  // Canal A2 - Vx
//    AdcaRegs.ADCSOC3CTL.bit.CHSEL  = 3;  // Canal A3 - Iy
//    AdcaRegs.ADCSOC4CTL.bit.CHSEL  = 4;  // Canal A4 - Vv
//    AdcaRegs.ADCSOC5CTL.bit.CHSEL  = 5;  // Canal A5 - Vcc1

    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcaRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcaRegs.ADCSOC5CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0; // disable INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 0; // disable INT2 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT3E = 0; // disable INT3 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT4E = 0; // disable INT4 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;     // end of SOC6 will set INT1 flag

EDIS;

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

EALLOW;     // ADCB
//    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 2;  // Canal B2 - Iz
//    AdcbRegs.ADCSOC1CTL.bit.CHSEL  = 3;  // Canal B3 - Ivcc2
//    AdcbRegs.ADCSOC2CTL.bit.CHSEL  = 4;  // Canal B4 - Vw
//    AdcbRegs.ADCSOC3CTL.bit.CHSEL  = 5;  // Canal B5 - Iw
//    AdcbRegs.ADCSOC4CTL.bit.CHSEL  = 14; // Canal 14 - Vcc2
//
//    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//
//    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0; // disable INT1 flag
//    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 0; // disable INT2 flag
//    AdcbRegs.ADCINTSEL3N4.bit.INT3E = 0; // disable INT3 flag
//    AdcbRegs.ADCINTSEL3N4.bit.INT4E = 0; // disable INT4 flag
//
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;     // end of SOC6 will set INT1 flag

EDIS;

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

EALLOW;     // ADCC
    AdccRegs.ADCSOC0CTL.bit.CHSEL  = 2;  // Canal C2 - Ix
//    AdccRegs.ADCSOC1CTL.bit.CHSEL  = 3;  // Canal C3 - Vz
//    AdccRegs.ADCSOC2CTL.bit.CHSEL  = 4;  // Canal C4 - Ivcc1
//    AdccRegs.ADCSOC3CTL.bit.CHSEL  = 5;  // Canal C5 - Vu
//    AdccRegs.ADCSOC4CTL.bit.CHSEL  = 15; // Canal 15 - Iu

    AdccRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdccRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdccRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdccRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
//    AdccRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles

    AdccRegs.ADCSOC0CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdccRegs.ADCSOC1CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdccRegs.ADCSOC2CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdccRegs.ADCSOC3CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
//    AdccRegs.ADCSOC4CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C

    AdccRegs.ADCINTSEL1N2.bit.INT1E = 0; // disable INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT2E = 0; // disable INT2 flag
    AdccRegs.ADCINTSEL3N4.bit.INT3E = 0; // disable INT3 flag
    AdccRegs.ADCINTSEL3N4.bit.INT4E = 0; // disable INT4 flag

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;     // end of SOC6 will set INT1 flag


EDIS;

EALLOW;
    //
    //enable ADCINT flags
    //
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.all = 0x000F;
    AdcbRegs.ADCINTFLGCLR.all = 0x000F;
    AdccRegs.ADCINTFLGCLR.all = 0x000F;
EDIS;

}


//###############CONFIGURAÇÃO DOS CANAIS DE PWM####################

void Setup_ePWM(void)
{
    EPwm1Regs.TBCTL.bit.CLKDIV=0;       //CLKDIV=1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm1Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm1Regs.TBPRD=125;                // 100 kHz
    EPwm1Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm1Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm1Regs.ETPS.bit.SOCAPRD=1;       // Generate pulse on first event
    EPwm1Regs.ETPS.bit.SOCBPRD=1;       // Generate pulse on first event
    EPwm1Regs.TBCTR = 0x0000;           // Clear counter


    EPwm1Regs.ETSEL.bit.SOCAEN=1;
    EPwm1Regs.ETSEL.bit.SOCASEL=1;
    EPwm1Regs.ETSEL.bit.SOCBEN=1;
    EPwm1Regs.ETSEL.bit.SOCBSEL=2;
    EPwm1Regs.CMPA.bit.CMPA = 1250;

    EPwm2Regs.TBCTL.bit.CLKDIV=0;       // CLKDIV=1
    EPwm2Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm2Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm2Regs.TBPRD=2500;               // 15 kHz
    EPwm2Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm2Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm2Regs.TBPHS.bit.TBPHS=0;        // 2*1250;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.TBCTR = 0x0000;           // Clear counter

    EPwm3Regs.TBCTL.bit.CLKDIV=0;       // CLKDIV=1
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm3Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm3Regs.TBPRD=2500;
    EPwm3Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm3Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm3Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.TBCTR = 0x0000;           // Clear counter

    EPwm4Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm4Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm4Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm4Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm4Regs.TBPRD=2500;
    EPwm4Regs.CMPA.bit.CMPA = 0;
    EPwm4Regs.TBCTR = 0x0000;           // Clear counter

    EPwm5Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPwm5Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm5Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm5Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm5Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm5Regs.TBPRD=2500;
    EPwm5Regs.CMPA.bit.CMPA = 0;

    EPwm6Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPwm6Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm6Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm6Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm6Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm6Regs.TBPRD=2500;
    EPwm6Regs.CMPA.bit.CMPA = 0;
    EPwm6Regs.TBCTR = 0x0000;           // Clear counter

    EPwm1Regs.TBCTL.bit.SYNCOSEL=1;

    EPwm2Regs.TBCTL.bit.PHSEN=1;
    EPwm2Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm2Regs.TBPHS.bit.TBPHS=0;     	// Fase

    EPwm3Regs.TBCTL.bit.PHSEN=1;
    EPwm3Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm3Regs.TBPHS.bit.TBPHS=0;

    EPwm4Regs.TBCTL.bit.PHSEN=1;
    EPwm4Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm4Regs.TBPHS.bit.TBPHS=0;

    EPwm5Regs.TBCTL.bit.PHSEN=1;
    EPwm5Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm5Regs.TBPHS.bit.TBPHS=0;

    EPwm6Regs.TBCTL.bit.PHSEN=1;
    EPwm6Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm6Regs.TBPHS.bit.TBPHS=0;

    //Tripzones EPWM1
EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm1Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm1Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm1Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm1Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm1Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm1Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm1Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm1Regs.TZCLR.bit.OST = 0;	    //
EDIS;

    //Tripzones EPWM2
EALLOW;
    EPwm2Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm2Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm2Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm2Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm2Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm2Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm2Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm2Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm2Regs.TZCLR.bit.OST = 0;	    //
EDIS;

    //Tripzones EPWM3
EALLOW;
    EPwm3Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm3Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm3Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm3Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm3Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm3Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm3Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm3Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm3Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM4
EALLOW;
    EPwm4Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm4Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm4Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm4Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm4Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm4Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm4Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm4Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm4Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM5
EALLOW;
    EPwm5Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm5Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm5Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm5Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm5Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm5Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm5Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm5Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm5Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM6
EALLOW;
    EPwm6Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm6Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm6Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPwm6Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm6Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm6Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm6Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm6Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm6Regs.TZCLR.bit.OST = 0;
EDIS;


    //Inicialização de pwm com zero
EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
EDIS;
}

//###############CONFIGURAÇÃO DO DO ENCODER####################

void Setup_EQEP(void)
{
	 //Code'+.++++++-s of the quadrature encoder configuration
     EQep1Regs.QDECCTL.bit.QSRC = 0;        //quadrature-count mode
     EQep1Regs.QDECCTL.bit.XCR = 0;         //2x resolution: count the rising edge/falling
     EQep1Regs.QEPCTL.bit.PCRM = 0;         //position counter reset on an index event
     EQep1Regs.QEPCTL.bit.QPEN = 1;         //eQEP position counter is enable
     EQep1Regs.QPOSMAX = 14399;             // 3600; 14399; //Maximum number of position counter ->  4*3600 - 1 = 14399


//     EQep1Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
//                                           // SYSCLKOUT
//     EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
//     EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
//     EQep1Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
//                                           // index event
//     EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
//     EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
//     EQep1Regs.QPOSMAX = 0xffffffff;
//     EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
//     EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
//     EQep1Regs.QCAPCTL.bit.CCPS = 6;       // 1/64 for CAP clock
//     EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
}


//#####################CONFIGURA SERIAL##########################
void SCIB_init()
{
	ScibRegs.SCICCR.all = 0x0027;   // 1 stop bit,  No loopback
	                                // ODD parity,8 char bits,
	                                // async mode, idle-line protocol

	ScibRegs.SCICTL1.all =0x0003;   // enable TX, RX, internal SCICLK,
	                                // Disable RX ERR, SLEEP, TXWAKE

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
	//
    // SCIA at 115200 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x35.
    //
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x0035;

	ScibRegs.SCICTL2.bit.TXINTENA = 1; // enable SCI-A Tx-ISR
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1; 	// enable SCI_A Rx-ISR ===INCLUIDO
	ScibRegs.SCIFFTX.all = 0xC060;	// bit 15 = 1 : relinquish from Reset
											// bit 14 = 1 : Enable FIFO
											// bit 6 = 1 :  CLR TXFFINT-Flag
											// bit 5 = 1 :  enable TX FIFO match
											// bit 4-0 :  TX-ISR, if TX FIFO is 0(empty)
	ScibRegs.SCIFFCT.all = 0x0000;	    // Set FIFO transfer delay to 0
	ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;  // re-enable transmit fifo operation
	ScibRegs.SCIFFRX.all = 0xE010;	// Rx interrupt level = 5
	ScibRegs.SCICTL1.all = 0x0023;	// Relinquish SCI from Reset

}

void SCIC_init()
{
    ScicRegs.SCICCR.all = 0x0027;   // 1 stop bit,  No loopback
                                    // ODD parity,8 char bits,
                                    // async mode, idle-line protocol

    ScicRegs.SCICTL1.all =0x0003;   // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    //
    // SCIA at 115200 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x35.
    //
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x0035;

    ScicRegs.SCICTL2.bit.TXINTENA = 1; // enable SCI-A Tx-ISR
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;    // enable SCI_A Rx-ISR ===INCLUIDO
    ScicRegs.SCIFFTX.all = 0xC060;  // bit 15 = 1 : relinquish from Reset
                                            // bit 14 = 1 : Enable FIFO
                                            // bit 6 = 1 :  CLR TXFFINT-Flag
                                            // bit 5 = 1 :  enable TX FIFO match
                                            // bit 4-0 :  TX-ISR, if TX FIFO is 0(empty)
    ScicRegs.SCIFFCT.all = 0x0000;  // Set FIFO transfer delay to 0
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;  // re-enable transmit fifo operation
    ScicRegs.SCIFFRX.all = 0xE010;  // Rx interrupt level = 5
    ScicRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset

}


//#############################################################################################

