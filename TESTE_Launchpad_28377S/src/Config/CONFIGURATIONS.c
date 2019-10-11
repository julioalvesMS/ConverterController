#include <src/Config/CONFIGURATIONS.h>     // Device Headerfile and Examples Include File


//################## ROTINAS CONFIGURAÇÃO DO FUNCIONAMENTO DOS PERIFÉRICOS#############################



#define ADC_TRIG_SOURCE         17   // Triggered by PWM1A

//############CONFIGURAÇÃO DOS CANAIS ANALÓGICOS ADC ###################

void Setup_ADC(void)
{

    Uint16 acqps;

EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

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
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 0;  // Canal A0 - Ix
    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = 1;  // Canal A1 - Vx
    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = 2;  // Canal A2 - Vu
    AdcaRegs.ADCSOC3CTL.bit.CHSEL  = 3;  // Canal A3 - Icc1
    AdcaRegs.ADCSOC4CTL.bit.CHSEL  = 4;  // Canal A4 - Vv
    AdcaRegs.ADCSOC5CTL.bit.CHSEL  = 5;  // Canal A5 - Iw
    AdcaRegs.ADCSOC6CTL.bit.CHSEL  = 14; // Canal 14 - Vcc2

    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC6CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0; // disable INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 0; // disable INT2 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT3E = 0; // disable INT3 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT4E = 0; // disable INT4 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 6;     // end of SOC6 will set INT1 flag

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
    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 0;  // Canal B0 - Iz
    AdcbRegs.ADCSOC1CTL.bit.CHSEL  = 1;  // Canal B1 - Vz
    AdcbRegs.ADCSOC2CTL.bit.CHSEL  = 2;  // Canal B2 - Iy
    AdcbRegs.ADCSOC3CTL.bit.CHSEL  = 3;  // Canal B3 - Vw
    AdcbRegs.ADCSOC4CTL.bit.CHSEL  = 4;  // Canal B4 - Icc2
    AdcbRegs.ADCSOC5CTL.bit.CHSEL  = 5;  // Canal B5 - Vcc1
    AdcbRegs.ADCSOC6CTL.bit.CHSEL  = 15; // Canal 15 - Iu

    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC6CTL.bit.ACQPS  = acqps;    // sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC6CTL.bit.TRIGSEL  = ADC_TRIG_SOURCE;  // trigger on ePWM1 SOCA/C

    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0; // disable INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 0; // disable INT2 flag
    AdcbRegs.ADCINTSEL3N4.bit.INT3E = 0; // disable INT3 flag
    AdcbRegs.ADCINTSEL3N4.bit.INT4E = 0; // disable INT4 flag

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 6;     // end of SOC6 will set INT1 flag

EDIS;


EALLOW;
    //
    //enable ADCINT flags
    //
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.all = 0x000F;
    AdcbRegs.ADCINTFLGCLR.all = 0x000F;
EDIS;

}


//###############CONFIGURAÇÃO DOS CANAIS DE PWM####################

void Setup_ePWM(void)
{
    //  EPWM1 (PWM7)
    EPWM1_Regs.TBCTL.bit.CLKDIV=0;       //CLKDIV=1
    EPWM1_Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPWM1_Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPWM1_Regs.TBPRD=2500;               // ~ 15 kHz
    EPWM1_Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPWM1_Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPWM1_Regs.ETPS.bit.SOCAPRD=1;       // Generate pulse on first event
    EPWM1_Regs.ETPS.bit.SOCBPRD=1;       // Generate pulse on first event
    EPWM1_Regs.TBCTR = 0x0000;           // Clear counter


    EPWM1_Regs.ETSEL.bit.SOCAEN=1;
    EPWM1_Regs.ETSEL.bit.SOCASEL=1;
    EPWM1_Regs.ETSEL.bit.SOCBEN=1;
    EPWM1_Regs.ETSEL.bit.SOCBSEL=2;
    EPWM1_Regs.CMPA.bit.CMPA = 1250;

    //
    //  EPWM2 (PWM8)
    //
    EPWM2_Regs.TBCTL.bit.CLKDIV=0;       // CLKDIV=1
    EPWM2_Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPWM2_Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPWM2_Regs.TBPRD=2500;               // 15 kHz
    EPWM2_Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPWM2_Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPWM2_Regs.TBPHS.bit.TBPHS=0;        // 2*1250;
    EPWM2_Regs.CMPA.bit.CMPA = 0;
    EPWM2_Regs.TBCTR = 0x0000;           // Clear counter

    //
    //  EPWM3 (PWM9)
    //
    EPWM3_Regs.TBCTL.bit.CLKDIV=0;       // CLKDIV=1
    EPWM3_Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPWM3_Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPWM3_Regs.TBPRD=2500;
    EPWM3_Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPWM3_Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPWM3_Regs.CMPA.bit.CMPA = 0;
    EPWM3_Regs.TBCTR = 0x0000;           // Clear counter

    //
    //  EPWM4 (PWM2)
    //
    EPWM4_Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPWM4_Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPWM4_Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPWM4_Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPWM4_Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPWM4_Regs.TBPRD=2500;
    EPWM4_Regs.CMPA.bit.CMPA = 0;
    EPWM4_Regs.TBCTR = 0x0000;           // Clear counter

    //
    //  EPWM5 (PWM6)
    //
    EPWM5_Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPWM5_Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPWM5_Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPWM5_Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPWM5_Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPWM5_Regs.TBPRD=2500;
    EPWM5_Regs.CMPA.bit.CMPA = 0;

    //
    //  EPWM6 (PWM10)
    //
    EPWM6_Regs.TBCTL.bit.CLKDIV=0;   	// CLKDIV=1
    EPWM6_Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPWM6_Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPWM6_Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPWM6_Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPWM6_Regs.TBPRD=2500;
    EPWM6_Regs.CMPA.bit.CMPA = 0;
    EPWM6_Regs.TBCTR = 0x0000;           // Clear counter

    EPWM1_Regs.TBCTL.bit.SYNCOSEL=1;

    EPWM2_Regs.TBCTL.bit.PHSEN=1;
    EPWM2_Regs.TBCTL.bit.SYNCOSEL=0;
    EPWM2_Regs.TBPHS.bit.TBPHS=0;     	// Fase

    EPWM3_Regs.TBCTL.bit.PHSEN=1;
    EPWM3_Regs.TBCTL.bit.SYNCOSEL=0;
    EPWM3_Regs.TBPHS.bit.TBPHS=0;

    EPWM4_Regs.TBCTL.bit.PHSEN=1;
    EPWM4_Regs.TBCTL.bit.SYNCOSEL=0;
    EPWM4_Regs.TBPHS.bit.TBPHS=0;

    EPWM5_Regs.TBCTL.bit.PHSEN=1;
    EPWM5_Regs.TBCTL.bit.SYNCOSEL=0;
    EPWM5_Regs.TBPHS.bit.TBPHS=0;

    EPWM6_Regs.TBCTL.bit.PHSEN=1;
    EPWM6_Regs.TBCTL.bit.SYNCOSEL=0;
    EPWM6_Regs.TBPHS.bit.TBPHS=0;

    //Tripzones EPWM1 (PWM7)
EALLOW;
    EPWM1_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM1_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM1_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM1_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM1_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM1_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM1_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM1_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM1_Regs.TZCLR.bit.OST = 0;	    //
EDIS;

    //Tripzones EPWM2 (PWM8)
EALLOW;
    EPWM2_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM2_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM2_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM2_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM2_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM2_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM2_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM2_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM2_Regs.TZCLR.bit.OST = 0;	    //
EDIS;

    //Tripzones EPWM3 (PWM9)
EALLOW;
    EPWM3_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM3_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM3_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM3_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM3_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM3_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM3_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM3_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM3_Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM4 (PWM2)
EALLOW;
    EPWM4_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM4_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM4_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM4_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM4_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM4_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM4_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM4_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM4_Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM5 (PWM6)
EALLOW;
    EPWM5_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM5_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM5_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM5_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM5_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM5_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM5_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM5_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM5_Regs.TZCLR.bit.OST = 0;
EDIS;

    //Tripzones EPWM6 (PWM10)
EALLOW;
    EPWM6_Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPWM6_Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPWM6_Regs.TZSEL.bit.OSHT1 = 0;	    // select TZ1 -
    EPWM6_Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPWM6_Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPWM6_Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPWM6_Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPWM6_Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPWM6_Regs.TZCLR.bit.OST = 0;
EDIS;


    //Inicialização de pwm com zero
EALLOW;
    EPWM1_Regs.TZFRC.bit.OST = 1;
    EPWM2_Regs.TZFRC.bit.OST = 1;
    EPWM3_Regs.TZFRC.bit.OST = 1;
    EPWM4_Regs.TZFRC.bit.OST = 1;
    EPWM5_Regs.TZFRC.bit.OST = 1;
    EPWM6_Regs.TZFRC.bit.OST = 1;
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
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x44.
    //
    // SCIA at 115200 baud
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x1A.
    //
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x001A;

	ScibRegs.SCICTL2.bit.TXINTENA = 1;      // enable SCI-A Tx-ISR
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1; 	// enable SCI_A Rx-ISR ===INCLUIDO
	ScibRegs.SCIFFTX.all = 0xC060;	        // bit 15 = 1 : relinquish from Reset
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
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x44.
    //
    // SCIA at 115200 baud
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x1A.
    //
    ScicRegs.SCIHBAUD.all = 0x00;
    ScicRegs.SCILBAUD.all = 0x1A;

    ScicRegs.SCICTL2.bit.TXINTENA = 1;      // enable SCI-A Tx-ISR
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;    // enable SCI_A Rx-ISR ===INCLUIDO
    ScicRegs.SCIFFTX.all = 0xC060;          // bit 15 = 1 : relinquish from Reset
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

