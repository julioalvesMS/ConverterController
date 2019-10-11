#include <src/Config/CONFIGURATIONS.h>     // Device Headerfile and Examples Include File


//################## ROTINAS CONFIGURAÇÃO DO FUNCIONAMENTO DOS PERIFÉRICOS#############################


//############CONFIGURAÇÃO DOS CANAIS ANALÓGICOS ADC ###################

void Setup_ADC(void)
{

    AdcRegs.ADCTRL1.all=0;
    AdcRegs.ADCTRL1.bit.ACQ_PS=7;      // 7 = 8 x ADCCLK
    AdcRegs.ADCTRL1.bit.SEQ_CASC=1;    // 1=cascaded sequencer
    AdcRegs.ADCTRL1.bit.CPS=0;         // divide by 1
    AdcRegs.ADCTRL1.bit.CONT_RUN=0;    // single run mode
    AdcRegs.ADCTRL2.all=0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;     // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1;   // 1=SEQ1 start from ePWM_SOCA trigger
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;     // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ=1;    // 1=SEQ1 start from ePWM_SOCA trigger
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;     // 0= interrupt after every end of sequence
    AdcRegs.ADCTRL3.bit.ADCCLKPS=1;         // ADC clock: FCLK = HSPCLK / 2 * ADCCLKPS
                                            // HSPCLK = 75MHz (see DSP2833x_SysCtrl.c)
                                            // FCLK = 12.5 MHz
    AdcRegs.ADCMAXCONV.all=0x0002;          // 3 conversões (ALTERADO: ORIGINAL DE 8)
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0x1;    // amostragem simultânea

    AdcRegs.ADCCHSELSEQ1.bit.CONV00=0;      // (A0,B0)=(Ix,Iy)=(result0,result1)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01=2;   // (A2,B2)=(Vx,Vz)=(result2,result3)
    AdcRegs.ADCCHSELSEQ1.bit.CONV02=3;   // (A3,B3)=(Vy,Vcc2)=(result4,result5)

//    AdcRegs.ADCCHSELSEQ1.bit.CONV00=0;   // (A0,B0)=(Ix,Iy)=(result0,result1)
//    AdcRegs.ADCCHSELSEQ1.bit.CONV01=1;   // (A1,B1)=(Iz,Ivcc2)=(result2,result3)
//    AdcRegs.ADCCHSELSEQ1.bit.CONV02=2;   // (A2,B2)=(Vx,Vz)=(result4,result5)
//    AdcRegs.ADCCHSELSEQ1.bit.CONV03=3;   // (A3,B3)=(Vy,Vcc2)=(result6,result7)
//    AdcRegs.ADCCHSELSEQ2.bit.CONV04=4;   // (A4,B4)=(Iu,Iv)=(result8,result9)
//    AdcRegs.ADCCHSELSEQ2.bit.CONV05=5;   // (A5,B5)=(Vu,Vv)=(result10,result11)
//    AdcRegs.ADCCHSELSEQ2.bit.CONV06=6;   // (A6,B6)=(Iw,Vw)=(result12,result13)
//    AdcRegs.ADCCHSELSEQ2.bit.CONV07=7;   // (A7,B7)=(Vcc1,Icc1)=(result14,result15)

}


//###############CONFIGURAÇÃO DOS CANAIS DE PWM####################

void Setup_ePWM(void)
{
    EPwm1Regs.TBCTL.bit.CLKDIV=0;       //CLKDIV=1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm1Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm1Regs.TBPRD=1250;               // ~ 20 kHz
    EPwm1Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm1Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm1Regs.ETPS.bit.SOCAPRD=1;       // Generate pulse on first event
    EPwm1Regs.ETPS.bit.SOCBPRD=1;       // Generate pulse on first event


    EPwm1Regs.ETSEL.bit.SOCAEN=1;
    EPwm1Regs.ETSEL.bit.SOCASEL=1;
    EPwm1Regs.ETSEL.bit.SOCBEN=1; //*
    EPwm1Regs.ETSEL.bit.SOCBSEL=2; //*
    EPwm1Regs.CMPA.half.CMPA = 0;

    EPwm2Regs.TBCTL.bit.CLKDIV=0;       //CLKDIV=1
    EPwm2Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm2Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm2Regs.TBPRD=2500;               // 15 kHz
    EPwm2Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm2Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm2Regs.TBPHS.half.TBPHS=0;       //2*1250;
    EPwm2Regs.CMPA.half.CMPA = 0;

    EPwm3Regs.TBCTL.bit.CLKDIV=0;       //CLKDIV=1
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=1;    // HSPCLKDIV =2
    EPwm3Regs.TBCTL.bit.CTRMODE=2;      // up down mode
    EPwm3Regs.TBPRD=2500;
    EPwm3Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
    EPwm3Regs.AQCTLB.all=0x0060;        // ZRO=set, PRD-clear
    EPwm3Regs.CMPA.half.CMPA = 0;

    EPwm4Regs.TBCTL.bit.CLKDIV=0;   	//CLKDIV=1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm4Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm4Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm4Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm4Regs.TBPRD=2500;
    EPwm4Regs.CMPA.half.CMPA = 0;

    EPwm5Regs.TBCTL.bit.CLKDIV=0;   	//CLKDIV=1
    EPwm5Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm5Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm5Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm5Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm5Regs.TBPRD=2500;
    EPwm5Regs.CMPA.half.CMPA = 0;

    EPwm6Regs.TBCTL.bit.CLKDIV=0;   	//CLKDIV=1
    EPwm6Regs.TBCTL.bit.HSPCLKDIV=1;	// HSPCLKDIV =2
    EPwm6Regs.TBCTL.bit.CTRMODE=2;  	// up down mode
    EPwm6Regs.AQCTLA.all=0x0090; 		// ZRO=set, PRD-clear
    EPwm6Regs.AQCTLB.all=0x0060; 		// ZRO=set, PRD-clear
    EPwm6Regs.TBPRD=2500;
    EPwm6Regs.CMPA.half.CMPA = 0;

    EPwm1Regs.TBCTL.bit.SYNCOSEL=1;

    EPwm2Regs.TBCTL.bit.PHSEN=1;
    EPwm2Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm2Regs.TBPHS.half.TBPHS=0;     	//fase

    EPwm3Regs.TBCTL.bit.PHSEN=1;
    EPwm3Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm3Regs.TBPHS.half.TBPHS=0;

    EPwm4Regs.TBCTL.bit.PHSEN=1;
    EPwm4Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm4Regs.TBPHS.half.TBPHS=0;

    EPwm5Regs.TBCTL.bit.PHSEN=1;
    EPwm5Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm5Regs.TBPHS.half.TBPHS=0;

    EPwm6Regs.TBCTL.bit.PHSEN=1;
    EPwm6Regs.TBCTL.bit.SYNCOSEL=0;
    EPwm6Regs.TBPHS.half.TBPHS=0;

    //Tripzones EPWM1
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = 2;		// force ePWM1A to zero
    EPwm1Regs.TZCTL.bit.TZB = 2;		// force ePWM1B to zero
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
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
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
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
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
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
    EPwm4Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
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
    EPwm5Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
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
    EPwm6Regs.TZSEL.bit.OSHT1 = 1;	    // select TZ1 -
    EPwm6Regs.TZSEL.bit.OSHT2 = 0;	    // select TZ2 -
    EPwm6Regs.TZSEL.bit.OSHT3 = 0;	    // select TZ3 -
    EPwm6Regs.TZSEL.bit.OSHT4 = 0;	    // select TZ4 -
    EPwm6Regs.TZSEL.bit.OSHT5 = 0;	    // select TZ5 -
    EPwm6Regs.TZSEL.bit.OSHT6 = 0;	    // select TZ6 -
    EPwm6Regs.TZCLR.bit.OST = 0;
    EDIS;


    EALLOW;								//Inicialização de pwm com zero
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
     EQep1Regs.QDECCTL.bit.QSRC=0;       //quadrature-count mode
     EQep1Regs.QDECCTL.bit.XCR=0;        //2x resolution: count the rising edge/falling
     EQep1Regs.QEPCTL.bit.PCRM=0;        //position counter reset on an index event
     EQep1Regs.QEPCTL.bit.QPEN=1;        //eQEP position counter is enable
     EQep1Regs.QPOSMAX = 14399;          // 3600; 14399; //Maximum number of position counter ->  4*3600 - 1 = 14399
}


//#####################CONFIGURA SERIAL##########################
void SCIA_init()
{
	SciaRegs.SCICCR.all = 0x0027;// 1 stop bit,  No loopback
		                                   	// ODD parity,8 char bits,
		                                   	// async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  	// enable TX, RX, internal SCICLK,
		                                   	// Disable RX ERR, SLEEP, TXWAKE

	// SYSCLOCKOUT = 150MHz; LSPCLK = 1/4 = 37.5 MHz
	// BRR = (LSPCLK / (9600 x 8)) -1
	// BRR = 487  gives 9605 Baud
	SciaRegs.SCIHBAUD    = 39 >> 8;		// Highbyte
	SciaRegs.SCILBAUD    = 39 & 0x00FF;	// Lowbyte
	SciaRegs.SCICTL2.bit.TXINTENA = 1; // enable SCI-A Tx-ISR
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1; 	// enable SCI_A Rx-ISR ===INCLUIDO
	SciaRegs.SCIFFTX.all = 0xC060;	// bit 15 = 1 : relinquish from Reset
											// bit 14 = 1 : Enable FIFO
											// bit 6 = 1 :  CLR TXFFINT-Flag
											// bit 5 = 1 :  enable TX FIFO match
											// bit 4-0 :  TX-ISR, if TX FIFO is 0(empty)
	SciaRegs.SCIFFCT.all = 0x0000;	// Set FIFO transfer delay to 0
	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  // re-enable transmit fifo operation
	SciaRegs.SCIFFRX.all = 0xE010;	// Rx interrupt level = 5
	SciaRegs.SCICTL1.all = 0x0023;	// Relinquish SCI from Reset

}


//#############################################################################################

