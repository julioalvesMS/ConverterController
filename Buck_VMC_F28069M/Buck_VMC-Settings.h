//----------------------------------------------------------------------------------
//	FILE:			{ProjectName}-Settings.h
//
//	Description:    This file contains the definitions for this project, and is 
//					linked to both {ProjectName}-Main.c and {ProjectName}-DPL-ISR.asm 
//					(where X is the project name).  
//
//	Type: 			Device Independent
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2010
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 9 April 2010 - MB
//----------------------------------------------------------------------------------

#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

//**********************************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//**********************************************************************************

#define CPU_SYS_CLOCK 90000

#define INCR_BUILD 2
#define	CNTRL_ISR_FREQ_RATIO	1

#define BUCK_PWM_SWITCHING_FREQUENCY 200
#define BUCK_PWM_PERIOD (CPU_SYS_CLOCK)/BUCK_PWM_SWITCHING_FREQUENCY

#define BUCK_PWM_NO       	4
#define ADC_TRIG_SOURCE   	11
#define ADC_PIN_VOUT	  	3
#define ADC_PIN_VIN	  		11
#define ADC_PIN_IL	  		4
#define ADC_PIN_IL_AVG 		12
#define ADC_IL_COMPARATOR 2

#define OUTPUT_VOLTAGE    2
#define VIN_MAX_SENSED    13.3
#define VOUT_MAX_SENSED   6.7
#define IL_MAX_SENSED     7.52
#define IL_TRIP_LEVEL     4

//==================================================================================
// Comp Settings
//----------------------------------------------------------------------------------

#define ACTIVE_COMP 1
#define CNTL_3p3z_A1_1 0.8285976581
#define CNTL_3p3z_A2_1 0.1714023419
#define CNTL_3p3z_A3_1 0.0000000000
#define CNTL_3p3z_B0_1 4.1703226660
#define CNTL_3p3z_B1_1 -5.9120992707
#define CNTL_3p3z_B2_1 1.9495912223
#define CNTL_3p3z_B3_1 0.0000000000
#define CNTL_3p3z_IMin_1 _IQ24(-0.1);
#define CNTL_3p3z_Max_1 _IQ24(0.9);
#define CNTL_3p3z_Min_1 _IQ24(0.0);

#define CNTL_3p3z_A1_2 0.9819683240
#define CNTL_3p3z_A2_2 0.0180316760
#define CNTL_3p3z_A3_2 0.0000000000
#define CNTL_3p3z_B0_2 5.9749793423
#define CNTL_3p3z_B1_2 -6.4434254835
#define CNTL_3p3z_B2_2 0.9774619791
#define CNTL_3p3z_B3_2 0.0000000000
#define CNTL_3p3z_IMin_2 _IQ24(-0.1);
#define CNTL_3p3z_Max_2 _IQ24(0.9);
#define CNTL_3p3z_Min_2 _IQ24(0.0);


#define CNTL_3p3z_A1_3 1.0640301741
#define CNTL_3p3z_A2_3 -0.0640301741
#define CNTL_3p3z_A3_3 0.0000000000
#define CNTL_3p3z_B0_3 0.3383153040
#define CNTL_3p3z_B1_3 -0.3564180834
#define CNTL_3p3z_B2_3 0.0512335741
#define CNTL_3p3z_B3_3 0.0000000000
#define CNTL_3p3z_IMin_3 _IQ24(-0.1);
#define CNTL_3p3z_Max_3 _IQ24(0.9);
#define CNTL_3p3z_Min_3 _IQ24(0.0);

#define CNTL_3p3z_A1_4 1.0640301741
#define CNTL_3p3z_A2_4 -0.0640301741
#define CNTL_3p3z_A3_4 0.0000000000
#define CNTL_3p3z_B0_4 0.3383153040
#define CNTL_3p3z_B1_4 -0.3564180834
#define CNTL_3p3z_B2_4 0.0512335741
#define CNTL_3p3z_B3_4 0.0000000000
#define CNTL_3p3z_IMin_4 _IQ24(-0.1);
#define CNTL_3p3z_Max_4 _IQ24(0.9);
#define CNTL_3p3z_Min_4 _IQ24(0.0);


#define CNTL_3p3z_A1_5 1.0729970749
#define CNTL_3p3z_A2_5 -0.0729970749
#define CNTL_3p3z_A3_5 0.0000000000
#define CNTL_3p3z_B0_5 0.0079766253
#define CNTL_3p3z_B1_5 -0.0091321655
#define CNTL_3p3z_B2_5 0.0019797754
#define CNTL_3p3z_B3_5 0.0000000000
#define CNTL_3p3z_IMin_5 _IQ24(-0.1);
#define CNTL_3p3z_Max_5 _IQ24(0.9);
#define CNTL_3p3z_Min_5 _IQ24(0.0);

//==================================================================================
// System Settings
//----------------------------------------------------------------------------------
//Add any system specific setting below
#define HistorySize 	8	// Number of samples averaged for use in GUI
#define DLOG_SIZE   	400

//==================================================================================
// Interrupt Framework options
//==================================================================================

#define EPWMn_DPL_ISR	1	// for EPWM triggered ISR set as 1
#define ADC_DPL_ISR	    0	// for ADC INT 1 triggered ISR set as 1 
#define CLAn_DPL_ISR	0	// for CLA Task n Triggered ISR set as 1

//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------
//#define EPWM1			1	// EPWM1 provides ISR trigger
//#define EPWM2			1 	// EPWM2 provides ISR trigger
//#define EPWM3			0	// EPWM3 provides ISR trigger
//#define EPWM4			0	// EPWM4 provides ISR trigger
//#define EPWM5			0	// EPWM5 provides ISR trigger
//#define EPWM6			0	// EPWM6 provides ISR trigger

#endif //_PROJSETTINGS_H


