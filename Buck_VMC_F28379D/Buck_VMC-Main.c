//----------------------------------------------------------------------------------
//	FILE:			Buck_VMC-Main.C
//
//	Description:	The file drives duty on PWM module selected by main.cfg
//
//	Version: 		1.0
//
//  Target:  		TMS320F2837xS
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// June 2015  - Created (HN)
//----------------------------------------------------------------------------------
//
// PLEASE READ - Useful notes about this Project

// Although this project is made up of several files, the most important ones are:
//	 "{ProjectName}-Main.C"	- this file
//		- Application Initialization, Peripheral config,
//		- Application management
//		- Slower background code loops and Task scheduling
//		- Time critical ISR functions
//	 "{ProjectName}-Settings.h"
//		- Global defines (settings) project selections are found here
//		- This file is referenced by both C and ASM files.
//
// Code is made up of sections, e.g. "FUNCTION PROTOTYPES", "VARIABLE DECLARATIONS" ,..etc
//	each section has FRAMEWORK and USER areas.
//  FRAMEWORK areas provide useful ready made "infrastructure" code which for the most part
//	does not need modification, e.g. Task scheduling, ISR call, GUI interface support,...etc
//  USER areas have functional example code which can be modified by USER to fit their appl.
//
// Code can be compiled with various build options (Incremental Builds IBx), these
//  options are selected in file main.cfg.
//----------------------------------------------------------------------------------

#include "Buck_VMC-Settings.h"
#include "F28x_Project.h"

#define   MATH_TYPE      1   //FLOAT_MATH

#include "DPlib.h"	
#include "IQmathLib.h"

#include "SFRA_F_Include.h"
#define SFRA_ISR_FREQ 200000
#define SFRA_FREQ_START 100
#define SFRA_FREQ_LENGTH 100
//SFRA step Multiply = 10^(1/No of steps per decade(35))
#define SFREQ_STEP_MULTIPLY (float)1.0680004

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Add protoypes of functions being used in the project here 
#ifdef FLASH		
	void InitFlash_Bank0();
#endif
void MemCopy();
void InitSysCtrl(void);
void InitPieVectTable(void);
void InitPieCtrl(void);
void SerialHostComms();
void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE);
void InitEPwm1Gpio(void);
void InitEPwm2Gpio(void);
void InitEPwm3Gpio(void);
void InitEPwm4Gpio(void);
void InitEPwm5Gpio(void);

//-------------------------------- DPLIB --------------------------------------------
void PWM_1ch_UpCntDB_ActivHIC_CNF(int16 n, int16 period, int16 mode, int16 phase);
interrupt void DPL_ISR_wFRA();

// -------------------------------- FRAMEWORK --------------------------------------
// State Machine function prototypes
//----------------------------------------------------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3
void A4(void);	//state A4

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3
void B4(void);	//state B4

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3
void C4(void);	//state C4

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch
//----------------------------------------------------------------------------------
// CPU Timer Definitions:
// Timer definitions based on 200MHz System Clock

	#define      mSec0_5          100000           // 0.5 mS
	#define      mSec0_75         150000           // 0.75 mS
	#define      mSec1            200000           // 1.0 mS
	#define      mSec2            400000           // 2.0 mS
	#define      mSec5            1000000          // 5.0 mS
	#define      mSec7_5          1500000          // 7.5 mS
	#define      mSec10           2000000          // 10 mS
	#define      mSec20           4000000          // 20 mS
	#define      mSec50           10000000         // 50 mS
	#define      mSec75           15000000         // 75 mS
	#define      mSec100          20000000         // 100 mS
	#define      mSec200          40000000         // 200 mS
	#define      mSec500          100000000        // 500 mS
	#define      mSec750          150000000        // 750 mS
	#define      mSec1000         200000000        // 1000 mS
	#define      mSec2000         400000000        // 2000 mS
	#define      mSec5000         1000000000       // 5000 mS

//===========================================================================

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] =
   { &EPwm1Regs,                                              //intentional: (ePWM[0] not used)
	 &EPwm1Regs,
	 &EPwm2Regs,
	 &EPwm3Regs,
	 &EPwm4Regs,
 	 &EPwm5Regs,
	 &EPwm6Regs,
	 &EPwm7Regs,
	 &EPwm8Regs,
 	 &EPwm9Regs,
	 &EPwm10Regs,
	 &EPwm11Regs,
	 &EPwm12Regs
   };

// Used to indirectly access all Comparator modules
volatile struct CMPSS_REGS *Cmpss[] =
   { &Cmpss1Regs,                                               //intentional: (Comp[0] not used)
 	 &Cmpss1Regs,
	 &Cmpss2Regs,
 	 &Cmpss3Regs,
	 &Cmpss4Regs,
	 &Cmpss5Regs,
	 &Cmpss6Regs,
	 &Cmpss7Regs
   };

volatile struct ADC_REGS *ADC[] =
   { &AdcaRegs,
	 &AdcaRegs,
	 &AdcbRegs,
	 &AdccRegs,
	 &AdcdRegs
   };

Uint16 Active_load_PRD = 35000;

// -------------------------------- FRAMEWORK --------------------------------------

int16	VTimer0[4];					// Virtual Timers slaved off CPU Timer 0
int16	VTimer1[4];					// Virtual Timers slaved off CPU Timer 1
int16	VTimer2[4];					// Virtual Timers slaved off CPU Timer 2
int16	SerialCommsTimer;
int16 	CommsOKflg;

// Used for running BackGround in flash, and ISR in RAM
//extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Used for ADC Configuration 
int 	ChSel1[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int 	ChSel2[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int     ACQPS1[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
int     ACQPS2[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
// ---------------------------------- USER -----------------------------------------
// ---------------------------- DPLIB Net Pointers ---------------------------------
// Declare net pointers that are used to connect the DP Lib Macros  here 
void ADC_SOC_CNF(int AdcNo,int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode,  Uint16 resolution, Uint16 signalmode);
extern void DacDrvCnf(int16 n, int16 DACval, int16 DACsrc, int16 RAMPsrc, int16 Slope_initial);

//CNTL2P2Z
volatile CNTL_2P2Z_F_C_Coeffs coeff1;
volatile CNTL_2P2Z_F_C_Vars var1;

//DLOG_1ch
volatile DLOG_1ch_F_C dlog1ch;

#if (BUCK_PWM_NO == 1)
	#define	BUCK_PWM_REG  EPwm1Regs
#elif (BUCK_PWM_NO == 2)
	#define	BUCK_PWM_REG  EPwm2Regs
#elif (BUCK_PWM_NO == 3)
	#define	BUCK_PWM_REG  EPwm3Regs
#elif (BUCK_PWM_NO == 4)
	#define	BUCK_PWM_REG  EPwm4Regs
#elif (BUCK_PWM_NO == 5)
	#define	BUCK_PWM_REG  EPwm5Regs
#endif

// ---------------------------- DPLIB Variables ---------------------------------
// Declare the net variables being used by the DP Lib Macro here 
volatile float Duty1A, Duty1A_Set;
volatile float Vref1 = 0;			// Output Set Voltage

float Itrip1 = 3276.0;				// 6A
int16 Pgain1_Gui = 100, Igain1_Gui = 5, Dgain1_Gui = 0;		// PID gains for the voltage loop
int16 pid2p2z_Gui= 1, coeff_change = 1;						// Flags for switching between PID and compensation designer based coeffiefients.
int16 No_2p2z = 0;											// Used to disable 2P2Z execution when control loop coefficients are being changed

float Pgain1, Igain1, Dgain1, Dmax1;

#pragma DATA_SECTION(DBUFF1,"DLOG_BUFF");
volatile float32 DBUFF1[DLOG_SIZE];

// System Flags
int16	FaultFlg1 = 0;			// Fault flag set on over current condition
int16 	ClearFault1 = 0;
int16	Active_LD1_EN = 0;

volatile float Adc_Vout1, Vout1_dlog = 0;
volatile float Vout1_slew_temp = 0, Duty1A_slew_temp = 0;			// Temp variable: used only if implementing slew rate control in the slower state machine
volatile float Vout1SetSlewed = 0.125, Duty1ASetSlewed = 0.125;		// Slewed set points
volatile float Vout1SlewRate = 0.0015, Duty1ASlewRate = 0.0015;		// Slew rate adjustment
volatile float Vout_Ref_wInj;

int16	Continuous_ON = 0, Start_Flag = 1;

// SFRA lib Object
SFRA_F SFRA1;
// SFRA Variables
float32 Plant_MagVect[SFRA_FREQ_LENGTH];
float32 Plant_PhaseVect[SFRA_FREQ_LENGTH];
float32 OL_MagVect[SFRA_FREQ_LENGTH];
float32 OL_PhaseVect[SFRA_FREQ_LENGTH];
float32 FreqVect[SFRA_FREQ_LENGTH];

//Flag for reinitializing SFRA variables
int16 initializationFlag;

//extern to access tables in ROM
extern long FPUsinTable[];

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - CCS WatchWindow / GUI support
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------- FRAMEWORK --------------------------------------

//GUI support variables
// sets a limit on the amount of external GUI controls - increase as necessary
int16 	*varSetTxtList[16];					//16 textbox controlled variables
int16 	*varSetBtnList[16];					//16 button controlled variables
int16 	*varSetSldrList[16];				//16 slider controlled variables
int16 	*varGetList[16];					//16 variables sendable to GUI
int32 	*arrayGetList[16];					//16 arrays sendable to GUI
Uint32 	*dataSetList[16];					//16 32-bit textbox or label controlled variables
int16  	LedBlinkCnt;

// ---------------------------------- USER -----------------------------------------

// Monitor ("Get")						// Display as:
float	Gui_Vin;
float	Gui_IL1;
float	Gui_Vout1;
float 	start_threshold = 0.02;
// Configure ("Set")
float	Gui_VSet1 = 0;
float	Gui_ItripSet = 6.0;

float Vout_max, Inv_Vout_max, Vin_max, IL_max, Inv_IL_max;

// Variables for background support only (no need to access)
int16	i;								// common use incrementer
Uint32	HistPtr, temp_Scratch; 			// Temp here means Temporary

// History arrays are used for Running Average calculation (boxcar filter)
// Used for CCS display and GUI only, not part of control loop processing
int16	Hist_Vin[HistorySize];
int16	Hist_IL1[HistorySize], Hist_IL2[HistorySize];
int16	Hist_Vout1[HistorySize], Hist_Vout2[HistorySize];

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{
//=================================================================================
//	INITIALIZATION - General
//=================================================================================

//-------------------------------- FRAMEWORK --------------------------------------
	//  Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F28M3Xx_SysCtrl.c file.
	InitSysCtrl();

    // Initialize the PLL control: PLLCR and CLKINDIV
    // F28_PLLCR and F28_CLKINDIV are defined in F2837xS_Examples.h
    // Note: The internal oscillator CANNOT be used as the PLL source if the
    // PLLSYSCLK is configured to frequencies above 194 MHz.
    InitSysPll(XTAL_OSC,IMULT_40,FMULT_1,PLLCLK_BY_2); 		//PLLSYSCLK = 10Mhz(OSCCLK) * 40 (IMULT) * 1 (FMULT) / 2

	// Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F28M3Xx_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
	// This function is found in F28M3Xx_PieVect.c.
	InitPieVectTable();

	SCIA_Init(50000000, 57600); // 50000000 is the LSPCLK or the Clock used for the SCI Module
								// 57600 is the Baudrate desired of the SCI module

// ***********************************
// Set up GPIOs
// **************************************
//--------------------------------------------------------------------------------------
// Configure GPIO outputs for PWM module selected in main.cfg
//--------------------------------------------------------------------------------------
	#if BUCK_PWM_NO == 1
		InitEPwm1Gpio();
	#elif BUCK_PWM_NO == 2
		InitEPwm2Gpio();
	#elif BUCK_PWM_NO == 3
		InitEPwm3Gpio();
	#elif BUCK_PWM_NO == 4
		InitEPwm4Gpio();
	#elif BUCK_PWM_NO == 5
		InitEPwm5Gpio();
	#endif
//--------------------------------------------------------------------------------------
// Configure GPIO output for PWM module that controls active load (EPWM10A)
//--------------------------------------------------------------------------------------
   EALLOW;
	/* Disable internal pull-up for the selected output pins
	   for reduced power consumption */
	// Pull-ups can be enabled or disabled by the user.
	// This will enable the pullups for the specified pins.
	// Comment out other unwanted lines.

	   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    // Disable pull-up on GPIO18 (EPWM10A)

	/* Configure EPWM-10 pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be EPWM10 functional pins.
	// Comment out other unwanted lines.

	   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;   // 5=EPWM10A. Configure GPIO18 as EPWM10A
	   GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as EPWM10A
//--------------------------------------------------------------------------------------
//  GPIO-13 - PIN FUNCTION = LED D10 on the F2837xS LaunchPad
//--------------------------------------------------------------------------------------
		GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;	// 0=GPIO,  1=Resv,  2=Resv,  3=Resv
		GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;		// 1=OUTput,  0=INput
		GpioDataRegs.GPASET.bit.GPIO13 = 1;		// Set High initially
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-84, GPIO-85 - PIN FUNCTION = SCITX and SCIRX
//--------------------------------------------------------------------------------------
		GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;	// 5=SCITX
		GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;	// 5=SCITX
		GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;	// 5=SCIRX
		GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;	// 5=SCIRX
//--------------------------------------------------------------------------------------
//  Connect the selected CMPSS module's output to TRIPIN5 through ePWM X-Bar
//--------------------------------------------------------------------------------------
		#if ADC_IL_COMPARATOR == 1
			EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.Mux0 = 0;		// CMPSS1 CTRIPH
			EPwmXbarRegs.TRIP4MUXENABLE.bit.Mux0 = 1;
		#elif ADC_IL_COMPARATOR == 2
			EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX2 = 0;		// CMPSS2 CTRIPH
			EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX2 = 1;
		#endif
//--------------------------------------------------------------------------------------
	EDIS;
// Timing sync for background loops
// Timer period definitions found in PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec1;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec10;	// B tasks
	CpuTimer2Regs.PRD.all =  mSec100;	// C tasks

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

	VTimer0[0] = 0;	
	VTimer1[0] = 0;
	VTimer2[0] = 0;
	LedBlinkCnt = 5;

	CommsOKflg = 0;
	SerialCommsTimer = 0;
	//"Set" variables
	// assign GUI Buttons to desired flag addresses
	varSetBtnList[0] = (int16*)&initializationFlag;

	//"Get" variables
	//---------------------------------------
	// assign a GUI "getable" parameter address
	varGetList[0] = (int16*)&(SFRA1.Vec_Length);		//int16
	varGetList[1] = (int16*)&(SFRA1.status);			//int16
	varGetList[2] = (int16*)&(SFRA1.FreqIndex);			//int16

	//"Setable" variables
	//----------------------------------------
	// assign GUI "setable" by Text parameter address
	dataSetList[0] = (Uint32*)&(SFRA1.Freq_Start);      //Float 32
	dataSetList[1] = (Uint32*)&(SFRA1.amplitude);	    //Int32
	dataSetList[2] = (Uint32*)&(SFRA1.Freq_Step);	    //Float32

	// assign a GUI "getable" parameter array address
	arrayGetList[0] = (int32*)FreqVect;			        //Float 32
	arrayGetList[1] = (int32*)OL_MagVect;			    //
	arrayGetList[2] = (int32*)OL_PhaseVect;		        //
	arrayGetList[3] = (int32*)Plant_MagVect;			//
	arrayGetList[4] = (int32*)Plant_PhaseVect;			//
	arrayGetList[5] = (int32*)&(SFRA1.Freq_Start);      //Float 32
	arrayGetList[6] = (int32*)&(SFRA1.amplitude);	    //Int32
	arrayGetList[7] = (int32*)&(SFRA1.Freq_Step);	    //Float32

	HistPtr = 0;
// ---------------------------------- USER -----------------------------------------
//  Put common initialization/variable definitions here

//Configure Scaling Constants
	Vout_max = 6.09;									// From excel spread sheet. Maximum possible output voltage.
	Vin_max = 12.09;
	IL_max = 6.83;
	Inv_IL_max = 0.146341463;
	Inv_Vout_max = 0.164179104;

	Duty1A_Set = 0.0;


	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

// --------------------------------------------------------------------------
//  Put peripheral initialization/variable definitions common to all builds here

// "Raw" (R) ADC measurement name defines
#define		Vout1R			AdcaResultRegs.ADCRESULT0	//
#define		IL1R			AdcaResultRegs.ADCRESULT1	//
#define		VinR	 		AdcbResultRegs.ADCRESULT0	//

// Channel Selection for Cascaded Sequencer
	ChSel1[0] = ADC_PIN_VOUT;				// Vout1
	ChSel1[1] = ADC_PIN_IL_AVG;				// IL average (heavily filtered)

	ChSel2[0] = ADC_PIN_VIN;				// Vin

	TrigSel1[0] = ADC_TRIG_SOURCE;			// Vout1 sampling
	TrigSel1[1] = ADC_TRIG_SOURCE;			// IL1 inductor current sampling
	TrigSel2[0] = ADC_TRIG_SOURCE;			// Vin sampling - Sampling triggered by the same trigger that triggers ADC-A conversions

	// Configure PWM for 200Khz (default) switching Frequency
	// Period Count= 100Mhz/200Khz = 500
	PWM_1ch_UpCntDB_ActivHIC_CNF(BUCK_PWM_NO, BUCK_PWM_PERIOD,1,0); 	// Master mode

// Configure the ADCs and power them up
	ADC_SOC_CNF(1, ChSel1,TrigSel1,ACQPS1, 16, 0,  ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);	// Mode= Start/Stop (0)
	ADC_SOC_CNF(2, ChSel2,TrigSel2,ACQPS2, 16, 0, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);	    // Mode= Start/Stop (0)

	ePWM[BUCK_PWM_NO]->ETSEL.bit.SOCAEN  =  1;
	ePWM[BUCK_PWM_NO]->ETSEL.bit.SOCASEL =  ET_CTRU_CMPA;  // Use CTR = CMPC/CMPA (CMPC selected in the next line of code)
	ePWM[BUCK_PWM_NO]->ETSEL.bit.SOCASELCMP = 1;		   // Enable event when CTR = CMPC
	ePWM[BUCK_PWM_NO]->ETPS.bit.SOCAPRD = 1; 	          // Generate pulse on 1st event

	DLOG_1ch_F_C_INIT(dlog1ch);
	dlog1ch.OutputBuff = DBUFF1;
	dlog1ch.PreScaler = 15;
	dlog1ch.TrigVal = 0.1;
	dlog1ch.Size = DLOG_SIZE;

//==================================================================================
//	INCREMENTAL BUILD OPTIONS - NOTE: selected via main.cfg
//==================================================================================
// ---------------------------------- USER -----------------------------------------
//----------------------------------------------------------------------
#if (INCR_BUILD == 1) 	// Open Loop Buck PWM Driver
//----------------------------------------------------------------------

// Initialize the net variable
	Duty1A = 0.0;

#endif // (INCR_BUILD == 1)

//----------------------------------------------------------------------
#if (INCR_BUILD == 2) 	// Closed Voltage Loop with SFRA
//----------------------------------------------------------------------

	CNTL_2P2Z_F_C_VAR_INIT(var1);
	CNTL_2P2Z_F_C_COEFF_INIT(coeff1);

	// Coefficients for the Voltage Loop
	#if (ACTIVE_COMP == 1)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_1);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_1);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_1);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_1);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_1);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_1;					  	//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_1; 					  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_1; 					//Clamp IMin
	#elif (ACTIVE_COMP == 2)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_2);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_2);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_2);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_2);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_2);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_2;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_2; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_2; 					//Clamp IMin
	#elif (ACTIVE_COMP == 3)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_3);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_3);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_3);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_3);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_3);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_3;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_3; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_3; 					//Clamp IMin
	#elif (ACTIVE_COMP == 4)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_4);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_4);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_4);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_4);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_4);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_4;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_4; 				  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_4; 					//Clamp IMin
	#elif (ACTIVE_COMP == 5)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_5);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_5);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_5);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_5);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_5);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_5;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_5; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_5; 					//Clamp IMin
	#endif

	var1.Ref = Vout_Ref_wInj;
	var1.Fdbk = Adc_Vout1;

// Initialize the net variable
	Duty1A = 0.0;

#endif // (INCR_BUILD == 2)


#if (INCR_BUILD == 3) 	// Closed Voltage Loop without SFRA
//----------------------------------------------------------------------
	CNTL_2P2Z_F_C_VAR_INIT(var1);
	CNTL_2P2Z_F_C_COEFF_INIT(coeff1);

	// Coefficients for the Voltage Loop
	#if (ACTIVE_COMP == 1)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_1);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_1);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_1);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_1);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_1);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_1;					  	//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_1; 					  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_1; 					//Clamp IMin
	#elif (ACTIVE_COMP == 2)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_2);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_2);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_2);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_2);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_2);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_2;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_2; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_2; 					//Clamp IMin
	#elif (ACTIVE_COMP == 3)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_3);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_3);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_3);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_3);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_3);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_3;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_3; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_3; 					//Clamp IMin
	#elif (ACTIVE_COMP == 4)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_4);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_4);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_4);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_4);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_4);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_4;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_4; 				  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_4; 					//Clamp IMin
	#elif (ACTIVE_COMP == 5)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_5);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_5);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_5);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_5);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_5);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_5;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_5; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_5; 					//Clamp IMin
	#endif

	var1.Ref = Vout_Ref_wInj;
	var1.Fdbk = Adc_Vout1;

// Initialize the net variable
		Duty1A = 0.0;

#endif // (INCR_BUILD == 3)

//----------------------------------------------------------------------
// Configure selected PWM, Comparator and DAC for over current protection
	EALLOW;	
// Define an event (DCAEVT1) based on Comparator Output
	ePWM[BUCK_PWM_NO]->DCTRIPSEL.bit.DCAHCOMPSEL = 4; 						// DCAH = TRIPIN5 (Comparator output)
	ePWM[BUCK_PWM_NO]->TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI; 					// DCAEVT1 = DCAH high(will become active
																			// as Comparator output goes high)
	ePWM[BUCK_PWM_NO]->DCACTL.bit.EVT1SRCSEL = DC_EVT1; 					// DCAEVT1 = DCAEVT1 (not filtered)
	ePWM[BUCK_PWM_NO]->DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;			// Take async path

// Enable DCAEVT1 as a one-shot source
	ePWM[BUCK_PWM_NO]->TZSEL.bit.DCAEVT1 = 1;

// What do we want the DCAEVT1 event to do?
	ePWM[BUCK_PWM_NO]->TZCTL.bit.TZA = TZ_FORCE_LO; 	// EPWMxA will go low
	ePWM[BUCK_PWM_NO]->TZCTL.bit.TZB = TZ_FORCE_LO; 	// EPWMxB will go low
	EDIS;

	DacDrvCnf(ADC_IL_COMPARATOR, (Uint16)Itrip1, 0, 2, 0);		// Selected comparator, DACval = Itrip1, DAC Source is DACval, Ramp Source = don't care, Slope = don't care

//===========================================================================
//Active load PWM drive configuration - EPwm10 is used
//===========================================================================
	//Time Base SubModule Register
	EPwm10Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;		// set Immediate load
	EPwm10Regs.TBPRD = Active_load_PRD;
	EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;		// divide by 4
	EPwm10Regs.TBCTL.bit.CLKDIV = 4;				// divide by 16
	EPwm10Regs.TBCTL.bit.PHSEN = TB_DISABLE;
	EPwm10Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; 	// sync "down-stream"

	// Counter compare submodule registers
	EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	// Action Qualifier SubModule Registers
	EPwm10Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm10Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm10Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm10Regs.AQCTLB.bit.CBU = AQ_CLEAR;

  	// Configure SOC event generation at PWM6 level - for slower ADC conversions
	EPwm10Regs.ETSEL.bit.SOCAEN  =  1;
	EPwm10Regs.ETSEL.bit.SOCASEL =  ET_CTR_ZERO; // Use CTR = ZRO events as trigger
    EPwm10Regs.ETPS.bit.SOCAPRD = 1; 	        // Generate pulse on 1st event

	EPwm10Regs.CMPA.bit.CMPA = 0;				// Active load disabled initially

//===========================================================================
//	SFRA Initialization
//===========================================================================
	//SFRA Object Initialization
	//Specify the injection amplitude
	SFRA1.amplitude= 0.005;
	//Specify the length of SFRA
	SFRA1.Vec_Length=SFRA_FREQ_LENGTH;
	//Specify the SFRA ISR Frequency
	SFRA1.ISR_Freq=SFRA_ISR_FREQ;
	//Specify the Start Frequency of the SFRA analysis
	SFRA1.Freq_Start=SFRA_FREQ_START;
	//Specify the Frequency Step
	SFRA1.Freq_Step=SFREQ_STEP_MULTIPLY;
	//Assign array location to Pointers in the SFRA object
	SFRA1.FreqVect=FreqVect;
	SFRA1.GH_MagVect=OL_MagVect;
	SFRA1.GH_PhaseVect=OL_PhaseVect;
	SFRA1.H_MagVect=Plant_MagVect;
	SFRA1.H_PhaseVect=Plant_PhaseVect;

	SFRA_F_INIT(&SFRA1);

//====================================================================================
// Start all enabled ePWM module clocks

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
//====================================================================================
	
//====================================================================================
// INTERRUPTS & ISR INITIALIZATION (best to run this section after other initialization)
//====================================================================================
// Set up C28x Interrupt

	EALLOW;
	// Configure selected PWM to issue interrupts
#if BUCK_PWM_NO==1
    PieVectTable.EPWM1_INT = &DPL_ISR_wFRA;      		// Map Interrupt
   	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      			// PIE level enable, Grp3 / Int1
#elif BUCK_PWM_NO == 2
    PieVectTable.EPWM2_INT = &DPL_ISR_wFRA;      		// Map Interrupt
   	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;      			// PIE level enable, Grp3 / Int2
#elif BUCK_PWM_NO == 3
    PieVectTable.EPWM3_INT = &DPL_ISR_wFRA;      		// Map Interrupt
   	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;      			// PIE level enable, Grp3 / Int3
#elif BUCK_PWM_NO == 4
    PieVectTable.EPWM4_INT = &DPL_ISR_wFRA;      		// Map Interrupt
   	PieCtrlRegs.PIEIER3.bit.INTx4 = 1;      			// PIE level enable, Grp3 / Int4
#elif BUCK_PWM_NO == 5
    PieVectTable.EPWM5_INT = &DPL_ISR_wFRA;      		// Map Interrupt
   	PieCtrlRegs.PIEIER3.bit.INTx5 = 1;      			// PIE level enable, Grp3 / Int5
#endif
   	ePWM[BUCK_PWM_NO]->ETSEL.bit.INTSEL = ET_CTRU_CMPA; // INT on CTR = CMPC/CMPA (CMPC selected in the nect line of code)
   	ePWM[BUCK_PWM_NO]->ETSEL.bit.INTSELCMP = 1;			// Enable event when CTR = CMPC
   	ePWM[BUCK_PWM_NO]->CMPC = (BUCK_PWM_PERIOD > 150)?(BUCK_PWM_PERIOD - 150):1;	// This CMPC value allows enough time to get a new ADC sample, and calculate and update the control action before the end of the PWM cycle.
   	ePWM[BUCK_PWM_NO]->ETSEL.bit.INTEN = 1;             // Enable INT

#if CNTRL_ISR_FREQ_RATIO ==	1
	 	 ePWM[BUCK_PWM_NO]->ETPS.bit.INTPRD = ET_1ST;   // Generate INT on every event
#elif	CNTRL_ISR_FREQ_RATIO ==	2
	 	 ePWM[BUCK_PWM_NO]->ETPS.bit.INTPRD = ET_2ND;   // Generate INT on every event
#elif	CNTRL_ISR_FREQ_RATIO ==	3
	 	 ePWM[BUCK_PWM_NO]->ETPS.bit.INTPRD = ET_3RD;   // Generate INT on every event
#endif

    IER |= M_INT3;		// Enable CPU INT3 connected to EPWM1-6 INTs:
    EINT;               // Enable Global interrupt INTM
    ERTM;               // Enable Global realtime interrupt DBGM
	EDIS;      

	ClearFault1 = 1;	// Clear any spurious trips

//=================================================================================
//	BACKGROUND (BG) LOOP
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================
		#if (INCR_BUILD != 3)
			if(initializationFlag == 1)
			{
				SFRA_F_INIT(&SFRA1);
				initializationFlag = 0;
				SFRA1.start = 1;
			}
		#endif
	}
} //END MAIN CODE

// Digital Power ISR
interrupt void DPL_ISR_wFRA()
{
	#if INCR_BUILD == 1
		Duty1A = SFRA_F_INJECT(Duty1ASetSlewed);		//Duty1ASetSlewed - Slewed Duty command (Duty1A_Set - set buy user)
		Adc_Vout1 = ADCDRV_1ch_F_C(Vout1R);				//Read Vout and convert to float
	#elif INCR_BUILD == 2
		Vout_Ref_wInj = SFRA_F_INJECT(Vout1SetSlewed);
		var1.Ref = Vout_Ref_wInj;
		Adc_Vout1 = ADCDRV_1ch_F_C(Vout1R);				//Read Vout and convert to float
		var1.Fdbk = Adc_Vout1;
		if (Start_Flag == 0 && No_2p2z == 0)
		{
			CNTL_2P2Z_F_C(coeff1,var1);
			Duty1A = var1.Out;
		}
	#elif INCR_BUILD == 3
		var1.Ref = Vout1SetSlewed;
		Adc_Vout1 = ADCDRV_1ch_F_C(Vout1R);				//Read Vout and convert to float
		var1.Fdbk = Adc_Vout1;
		if (Start_Flag == 0 && No_2p2z == 0)
		{
			CNTL_2P2Z_F_C(coeff1,var1);
			Duty1A = var1.Out;
		}
	#endif

	PWMDRV_1ch_F_C(BUCK_PWM_REG, BUCK_PWM_PERIOD, Duty1A);

	#if INCR_BUILD != 3
		SFRA_F_COLLECT(&Duty1A,&Adc_Vout1);
	#endif

	Vout1_dlog = Adc_Vout1*Vout_max;
	dlog1ch.Input = Vout1_dlog;
	DLOG_1ch_F_C(dlog1ch);

	BUCK_PWM_REG.ETCLR.bit.INT=1;
	PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;		// used by SciCommsGui_32bit.c
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;		// Back to State A0
}

//=================================================================================
//	A - TASKS
//=================================================================================
//---------------------------------------------------------------------------
void A1(void) // Update coefficients. Update overcurrent trip level.
//---------------------------------------------------------------------------
{
	Itrip1 = ((float) Gui_ItripSet * (float)Inv_IL_max * 4096.0);	 		// Scale to Q12
	EALLOW;
		Cmpss[ADC_IL_COMPARATOR]->DACHVALS.bit.DACVAL = (Uint16)Itrip1;		// DAC Value is in Q12
	EDIS;

	Pgain1 = (Pgain1_Gui >= 1000)?((float)1000):((float)Pgain1_Gui*(0.001)); // Clamp to 1000 and convert to per unit float value
	Igain1 = (Igain1_Gui >= 1000)?((float)1000):((float)Igain1_Gui*(0.001)); // Clamp to 1000 and convert to per unit float value
	Dgain1 = (Dgain1_Gui >= 1000)?((float)1000):((float)Dgain1_Gui*(0.001)); // Clamp to 1000 and convert to per unit float value

  if (coeff_change == 1)
   {
		No_2p2z = 1;	// Used to disable 2P2Z execution when coefficients are being changed

	if (pid2p2z_Gui == 0)
		{
		// Voltage loop coefficient update
		coeff1.Coeff_B2   = Dgain1;                            	// B2
		coeff1.Coeff_B1   = Igain1 - Pgain1 - Dgain1 - Dgain1;  	// B1
		coeff1.Coeff_B0   = Pgain1 + Igain1 + Dgain1;      		// B0
		coeff1.Coeff_A2   = 0.0;                              		// A2
		coeff1.Coeff_A1   = 1.0;                       		// A1
		}
	else
		{
		// Coefficients for the Voltage Loop
	#if (ACTIVE_COMP == 1)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_1);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_1);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_1);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_1);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_1);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_1;					  	//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_1; 					  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_1; 					//Clamp IMin
	#elif (ACTIVE_COMP == 2)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_2);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_2);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_2);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_2);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_2);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_2;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_2; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_2; 					//Clamp IMin
	#elif (ACTIVE_COMP == 3)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_3);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_3);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_3);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_3);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_3);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_3;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_3; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_3; 					//Clamp IMin
	#elif (ACTIVE_COMP == 4)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_4);                // B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_4);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_4);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_4);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_4);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_4;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_4; 				  	//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_4; 					//Clamp IMin
	#elif (ACTIVE_COMP == 5)
		// Coefficient init	--- Coeeficient values in Q26
		coeff1.Coeff_B2   = (float)(CNTL_3p3z_B2_5);               	// B2
		coeff1.Coeff_B1   = (float)(CNTL_3p3z_B1_5);  				// B1
		coeff1.Coeff_B0   = (float)(CNTL_3p3z_B0_5);      			// B0
		coeff1.Coeff_A2   = (float)(CNTL_3p3z_A2_5);                // A2
		coeff1.Coeff_A1   = (float)(CNTL_3p3z_A1_5);                // A1
		coeff1.Max   = (float)CNTL_3p3z_Max_5;					//Clamp Hi
		coeff1.Min   = (float)CNTL_3p3z_Min_5; 					//Clamp Min
		coeff1.IMin  = (float)CNTL_3p3z_IMin_5; 					//Clamp IMin
	#endif

		}
   No_2p2z = 0;
   coeff_change = 0;
  }
	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void)   // Slew Rate. Comms.
//-----------------------------------------------------------------
{	 
	// This is an example code for implementing the slew rate control in
	// a slower state machine
	// Duty1ASlewRate should be set as a positive value
#if INCR_BUILD == 1

	if (Duty1A_Set > (float)0.45)							// Clamp Duty command to 0.45 (Q24)
		Duty1A_Set = 0.45;

	Duty1A_slew_temp = Duty1A_Set - Duty1ASetSlewed;

	if (Duty1A_slew_temp >= Duty1ASlewRate) 			// Positive Command
	{
		Duty1ASetSlewed = Duty1ASetSlewed + Duty1ASlewRate;
	}
	else
		{
		if ((-1)*(Duty1A_slew_temp) >= Duty1ASlewRate) 	// Negative Command
			{
			Duty1ASetSlewed = Duty1ASetSlewed - Duty1ASlewRate;
			}
		else	Duty1ASetSlewed = Duty1A_Set;
		}
#else
	// Vout1SlewRate should be set as a positive value

	Vout1_slew_temp = Vref1 - Vout1SetSlewed;

	if (Vout1_slew_temp >= Vout1SlewRate) 				// Positive Command
	{
		Vout1SetSlewed = Vout1SetSlewed + Vout1SlewRate;
	}
	else
		{
		if ((-1)*(Vout1_slew_temp) >= Vout1SlewRate)	// Negative Command
			{
			Vout1SetSlewed = Vout1SetSlewed - Vout1SlewRate;
			}
		else	Vout1SetSlewed = Vref1;
		}
#endif

	SerialHostComms();

	//-------------------
	A_Task_Ptr = &A3;

	//-------------------
}

//-----------------------------------------
void A3(void)	// Active Load Enable/Disable
//-----------------------------------------
{
	if (Active_LD1_EN == 1)
	{
		if (Continuous_ON)
		{
			EPwm10Regs.CMPA.bit.CMPA = Active_load_PRD;
//			Gui_dlog_trig1 = Gui_Vout1;
			dlog1ch.TrigVal = Gui_Vout1;

		}
		else
		{
			EPwm10Regs.CMPA.bit.CMPA = Active_load_PRD>>1;
//			Gui_dlog_trig1 = Gui_Vout1 + 205;		//Vout + 0.1V = 0.1*2^12
			dlog1ch.TrigVal = Gui_Vout1 + 0.02;
		}
	}
	else
	{
			EPwm10Regs.CMPA.bit.CMPA = 0;
//			Gui_dlog_trig1 = Gui_Vout1;
			dlog1ch.TrigVal = Gui_Vout1;
	}			

	if (Gui_Vout1 < (0.1))			// <0.1V
//		Gui_dlog_trig1 = 245;	// 0.06V
		dlog1ch.TrigVal = 0.02;		// 0.05V
	
	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A4;
	//-----------------
}


//----------------------------------------------------------
void A4(void) 
//---------------------------------------------------------
{
	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}


//=================================================================================
//	B - TASKS
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Instrumentation
//----------------------------------------
{
// Please refer to DP_BoosterPack_Calculations.xls
	HistPtr++;
	if (HistPtr >= 8)	HistPtr = 0;

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------
	Hist_Vin[HistPtr] = VinR; 			// Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vin[i]; // Q12 * 8 = Q15
	temp_Scratch = temp_Scratch>>3;											   // Q12 i.e. averaged over 8 samples
	Gui_Vin = (float) temp_Scratch * INV_4096 * Vin_max;

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------   
	Hist_Vout1[HistPtr] = Vout1R; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vout1[i]; // Q12 * 8 = Q15
	temp_Scratch = temp_Scratch>>3;											  	 // Q12 i.e. averaged over 8 samples
	Gui_Vout1 = (float) temp_Scratch * INV_4096 * Vout_max;

//BoxCar Averages - Input Raw samples into History arrays 
//----------------------------------------------------------------
	Hist_IL1[HistPtr] = IL1R; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_IL1[i]; // Q12 * 8 = Q15
	temp_Scratch = temp_Scratch>>3;											   // Q12 i.e. averaged over 8 samples
	Gui_IL1 = (float) temp_Scratch * INV_4096 * IL_max ;

// Voltage command
	if (Gui_VSet1 < 6.0)				// Command voltage < 6V
		Vref1 = ((float)Gui_VSet1*(float)Inv_Vout_max);
//	else - If voltage command is > 6V, maintain the old output voltage value

	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;	
	//-----------------
}

//----------------------------------------
void B2(void) // Blink LED
//----------------------------------------
{
	if(LedBlinkCnt==0)
		{
			GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;	//turn on/off LED D10 on the F2837xS LaunchPad
			LedBlinkCnt=5;
		}
	else
			LedBlinkCnt--;

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void)  // Call SFRA
//----------------------------------------
{
	#if (INCR_BUILD != 3)
		SFRA_F_BACKGROUND(&SFRA1);
	#endif
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B4
	B_Task_Ptr = &B4;	
	//-----------------
}

//----------------------------------------
void B4(void) //  SPARE
//----------------------------------------
{
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;	
	//-----------------
}


//=================================================================================
//	C - TASKS
//=================================================================================
//--------------------------------- USER ------------------------------------------
//------------------------------------------------------
void C1(void) 	 // Fault Management
//------------------------------------------------------
{
	if ( (*ePWM[BUCK_PWM_NO]).TZFLG.bit.OST == 1 )
		FaultFlg1 = 1;
	else FaultFlg1 = 0;

	if (ClearFault1 == 1)
	{
		EALLOW;
		(*ePWM[BUCK_PWM_NO]).TZCLR.bit.OST = 1;
		EDIS;		
		ClearFault1 = 0;
	}

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------

}
//----------------------------------------
void C2(void) // Code to check switch position - open loop (OL) Excitation
//----------------------------------------
{
	#if	(INCR_BUILD != 1)
	  if (Start_Flag == 1)				// Execute only till the first time switch is turned ON
		  Duty1A = 0.05;				// OL excitation
	#endif

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C3
	C_Task_Ptr = &C3;	
	//-----------------
}


//-----------------------------------------
void C3(void) // Code to check switch position - Response
//-----------------------------------------
{
	#if	(INCR_BUILD != 1)
	  if (Start_Flag == 1)				// Execute only till the first time switch is turned ON
		{
		  if (Gui_Vout1 > (float)start_threshold) 			// Output voltage > 0.025V - Switch SW1 is in ON position
		  {
			  if (OUTPUT_VOLTAGE < 6)	// Max output voltage restricted to 6V
				  Gui_VSet1 = (float)OUTPUT_VOLTAGE;
			  else	Gui_VSet1 = 0.0;
			  Start_Flag = 0;
		  }
		  Duty1A = 0.0;					// Turn OFF OL excitation
		}
	#endif
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C4
	C_Task_Ptr = &C4;
	//-----------------
}


//-----------------------------------------
void C4(void) //  SPARE
//-----------------------------------------
{
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C1
	C_Task_Ptr = &C1;	
	//-----------------
}
