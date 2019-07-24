//----------------------------------------------------------------------------------
//	FILE:			Buck_VMC-Main.C
//
//	Description:	The file drives duty on PWM module selected by main.cfg
//
//	Version: 		1.0
//
//  Target:  		TMS320F2806x
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// February 2015  - Created (HN)
//----------------------------------------------------------------------------------
//
// PLEASE READ - Useful notes about this Project

// Although this project is made up of several files, the most important ones are:
//	 "{ProjectName}-Main.C"	- this file
//		- Application Initialization, Peripheral config,
//		- Application management
//		- Slower background code loops and Task scheduling
//	 "{ProjectName}-DevInit_F28xxx.C
//		- Device Initialization, e.g. Clock, PLL, WD, GPIO mapping
//		- Peripheral clock enables
//		- DevInit file will differ per each F28xxx device series, e.g. F280x, F2833x,
//	 "{ProjectName}-DPL-ISR.asm
//		- Assembly level library Macros and any cycle critical functions are found here
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
#include "PeripheralHeaderIncludes.h"
#include "F2806x_EPWM_defines.h"

#define   MATH_TYPE      0   //IQ_MATH
#include "DPlib.h"	
#include "IQmathLib.h"

#include "SFRA_IQ_Include.h"
#define SFRA_ISR_FREQ 200000
#define SFRA_FREQ_START 100
#define SFRA_FREQ_LENGTH 100
//SFRA step Multiply = 10^(1/No of steps per decade(40))
#define SFREQ_STEP_MULTIPLY (float)1.0680004

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Add protoypes of functions being used in the project here 
void DeviceInit(void);
#ifdef FLASH		
	void InitFlash();
#endif
void MemCopy();

void SerialHostComms();
void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE);

//-------------------------------- DPLIB --------------------------------------------
void PWM_1ch_UpCntDB_ActivHIC_CNF(int16 n, int16 period, int16 mode, int16 phase);
extern interrupt void DPL_ISR1(void);

// DLOG: data logger module
// Initalize memory buffers
static inline void DLOG_BuffInit(volatile int16 *buf, unsigned int buflen)
{
	int i=0;
	while(i<buflen)
	{
			buf[i++]=0;
	}
}

void DPL_Func();

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
	 #if (!DSP2802x_DEVICE_H)
 	 &EPwm5Regs,
	 &EPwm6Regs,
	 #if (DSP2803x_DEVICE_H || DSP2804x_DEVICE_H || F2806x_DEVICE_H)
	 &EPwm7Regs,
	 #if (DSP2804x_DEVICE_H || F2806x_DEVICE_H)
	 &EPwm8Regs
	 #endif
	 #endif
	 #endif
   };

// Used to indirectly access all Comparator modules
volatile struct COMP_REGS *Comp[] =
   { &Comp1Regs,                                               //intentional: (Comp[0] not used)
 	 &Comp1Regs,
	 &Comp2Regs,
	 #if (DSP2803x_DEVICE_H || F2806x_DEVICE_H)
	 &Comp3Regs
	 #endif
   };

Uint16 Active_load_PRD = 45000;
// -------------------------------- FRAMEWORK --------------------------------------

int16	VTimer0[4];					// Virtual Timers slaved off CPU Timer 0
int16	VTimer1[4];					// Virtual Timers slaved off CPU Timer 1
int16	VTimer2[4];					// Virtual Timers slaved off CPU Timer 2
int16	SerialCommsTimer;
int16 	CommsOKflg;

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Used for ADC Configuration 
int 	ChSel[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int     ACQPS[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
// ---------------------------------- USER -----------------------------------------
// ---------------------------- DPLIB Net Pointers ---------------------------------
// Declare net pointers that are used to connect the DP Lib Macros  here 
void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);
extern void DacDrvCnf(int16 n, int16 DACval, int16 DACsrc, int16 RAMPsrc, int16 Slope_initial);

// PWMDRV_1ch
extern volatile long *PWMDRV_1ch_Duty1;
extern volatile long *PWMDRV_1ch_Duty2;
extern volatile long *PWMDRV_1ch_Duty3;
extern volatile long *PWMDRV_1ch_Duty4;
extern volatile long *PWMDRV_1ch_Duty5;

// ADCDRV_1ch 
extern volatile long *ADCDRV_1ch_Rlt1; 	

//CNTL2P2Z
extern volatile long *CNTL_2P2Z_Ref1, *CNTL_2P2Z_Out1, *CNTL_2P2Z_Fdbk1;
extern volatile long *CNTL_2P2Z_Coef1;

//DLOG_1ch
extern volatile long *DLOG_1ch_Input1;
extern volatile int16 *DLOG_1ch_OutputBuff1;
extern volatile long DLOG_1ch_TrigVal1;
extern volatile int16 DLOG_1ch_PreScalar1;
extern volatile int16 DLOG_1ch_Size1;

// ---------------------------- DPLIB Variables ---------------------------------
// Declare the net variables being used by the DP Lib Macro here 
volatile long Duty1A, Duty1A_Set, temp = 0;

volatile long Vref1 = 0;			// Output Set Voltage

int16 Itrip1 = _IQ15(0.8);
int16 Pgain1_Gui = 100, Igain1_Gui = 5, Dgain1_Gui = 0;		// PID gains for the voltage loop
int16 pid2p2z_Gui= 1, coeff_change = 1;						// Flags for switching between PID and compensation designer based coeffiefients.
int16 No_2p2z = 0;											// Used to disable 2P2Z execution when control loop coefficients are being changed

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct1, "CNTL_2P2Z_Coef"); 
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;
long Pgain1, Igain1, Dgain1, Dmax1;	  	

#pragma DATA_SECTION(DBUFF1,"DLOG_BUFF");
volatile int16 DBUFF1[DLOG_SIZE];

// System Flags
int16	FaultFlg1 = 0;			// Fault flag set on over current condition
int16 	ClearFault1 = 0;
int16	Active_LD1_EN = 0;

volatile long Adc_Vout1, Vout1_dlog = 0;

volatile long Vout1_slew_temp = 0, Duty1A_slew_temp = 0;			// Temp variable: used only if implementing
																	// slew rate control in the slower state machine
volatile long Vout1SetSlewed = 2093568, Duty1ASetSlewed = 2093568;	// Slewed set point
volatile long Vout1SlewRate = 25600, Duty1ASlewRate = 25600;		// Slew rate adjustment

volatile long Vout_Ref_wInj;

int16	Continuous_ON = 0, Start_Flag = 1;

// SFRA lib Object
SFRA_IQ SFRA1;
// SFRA Variables
int32 Plant_MagVect[SFRA_FREQ_LENGTH];
int32 Plant_PhaseVect[SFRA_FREQ_LENGTH];
int32 OL_MagVect[SFRA_FREQ_LENGTH];
int32 OL_PhaseVect[SFRA_FREQ_LENGTH];
float32 FreqVect[SFRA_FREQ_LENGTH];

//Flag for reinitializing SFRA variables
int16 initializationFlag;

//extern to access tables in ROM
extern long IQsinTable[];

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
int16	Gui_Vin;							// Q5
int16	Gui_IL1;							// Q12
int16	Gui_Vout1;							// Q10

// Configure ("Set")
int16	Gui_VSet1 = 0;						// Q11
int16	Gui_dlog_trig1 = 0;					// Q11
int16	Gui_ItripSet = 24576;				// Q12 - Set to 6A

//Scaling Constants (values found via spreadsheet; exact value calibrated per board)
int16	K_Vin;								// Q15
int16	K_IL1;								// Q15
int16	K_Vout1;							// Q15 

int16	iK_IL1;								// Q14
int16	iK_Vout1;							// Q14

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

	// The DeviceInit() configures the clocks and pin mux registers 
	// The function is declared in {ProjectName}-DevInit_F2806x.c,
	// Please ensure/edit that all the desired components pin muxes 
	// are configured properly that clocks for the peripherals used
	// are enabled, for example the individual PWM clock must be enabled 
	// along with the Time Base Clock 

	DeviceInit();	// Device Life support & GPIO

//-------------------------------- FRAMEWORK --------------------------------------

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler with -d FLASH

#ifdef FLASH		
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

	SCIA_Init(22500000, 57600); // 22500000 is the LSPCLK or the Clock used for the SCI Module
								// 57600 is the Baudrate desired of the SCI module

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
	varGetList[0] = (int16*)&(SFRA1.Vec_Length);			//int16
	varGetList[1] = (int16*)&(SFRA1.status);			    //int16
	varGetList[2] = (int16*)&(SFRA1.FreqIndex);				//int16

	//"Setable" variables
	//----------------------------------------
	// assign GUI "setable" by Text parameter address
	dataSetList[0] = (Uint32*)&(SFRA1.Freq_Start);      //Float 32
	dataSetList[1] = (Uint32*)&(SFRA1.amplitude);	   //Int32
	dataSetList[2] = (Uint32*)&(SFRA1.Freq_Step);	   //Float32

	// assign a GUI "getable" parameter array address
	arrayGetList[0] = (int32*)FreqVect;			        //Float 32
	arrayGetList[1] = (int32*)OL_MagVect;			    //
	arrayGetList[2] = (int32*)OL_PhaseVect;		        //
	arrayGetList[3] = (int32*)Plant_MagVect;			//
	arrayGetList[4] = (int32*)Plant_PhaseVect;			//
	arrayGetList[5] = (int32*)&(SFRA1.Freq_Start);      //Float 32
	arrayGetList[6] = (int32*)&(SFRA1.amplitude);	   //Int32
	arrayGetList[7] = (int32*)&(SFRA1.Freq_Step);	   //Float32

	HistPtr = 0;
	
// ---------------------------------- USER -----------------------------------------
//  Put common initialization/variable definitions here

//Configure Scaling Constants
	K_Vin = 27238;									// 0.831 in Q15 (see excel spreadsheet)
	K_IL1 = 30788;									// 0.940 in Q15 (see excel spreadsheet)
	K_Vout1 = 27443;								// 0.838 in Q15 (see excel spreadsheet)
	
	iK_Vout1 = 19563;								// 1.194 in Q14 (see excel spreadsheet)
	iK_IL1 = 17438;									// 1.064 in Q14 (see excel spreadsheet)

	Duty1A_Set = 0;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; 
	EDIS;

// --------------------------------------------------------------------------
//  Put peripheral initialization/variable definitions common to all builds here

// "Raw" (R) ADC measurement name defines
#define		Vout1R			AdcResult.ADCRESULT1	//
#define		IL1R			AdcResult.ADCRESULT2	//
#define		VinR	 		AdcResult.ADCRESULT3	//

// Channel Selection for Cascaded Sequencer
	ChSel[0] = ADC_PIN_VOUT;				// Vout1 - Dummy
	ChSel[1] = ADC_PIN_VOUT;				// Vout1
	ChSel[2] = ADC_PIN_IL_AVG;				// IL average (heavily filtered)
	ChSel[3] = ADC_PIN_VIN;					// Vin

	TrigSel[0] = ADC_TRIG_SOURCE;			// Vout1 sampling - Dummy
	TrigSel[1] = ADC_TRIG_SOURCE; 			// Vout1 sampling - Sampled at PWM switching frequency
	TrigSel[2] = ADC_TRIG_SOURCE;			// IL1 inductor current sampling - Sampled at PWM switching frequency
	TrigSel[3] = 15;						// Vin sampling - EPWM6 SOCA - Sampled at a slower rate

	// Configure PWM for 200Khz (default) switching Frequency
	// Period Count= 90Mhz/200Khz = 450
	PWM_1ch_UpCntDB_ActivHIC_CNF(BUCK_PWM_NO, BUCK_PWM_PERIOD,1,0); 	// Master mode

	// Configure ADC
	EALLOW;
	AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;		// Divide ADC input clock by 2 for ADC
	EDIS;

	ADC_SOC_CNF(ChSel,TrigSel,ACQPS, 16, 0);// ACQPS=8, No ADC channel triggers an interrupt IntChSel > 15, Mode= Start/Stop (0)

	ePWM[BUCK_PWM_NO]->ETSEL.bit.SOCAEN  =  1;
	ePWM[BUCK_PWM_NO]->ETSEL.bit.SOCASEL = ET_CTR_ZERO;   // Use CTR = ZRO events as trigger
	ePWM[BUCK_PWM_NO]->ETPS.bit.SOCAPRD = 1; 	          // Generate pulse on 1st event

	// Digital Power library initialization
	DPL_Init();


//==================================================================================
//	INCREMENTAL BUILD OPTIONS - NOTE: selected via main.cfg
//==================================================================================
// ---------------------------------- USER -----------------------------------------
//----------------------------------------------------------------------
#if (INCR_BUILD == 1) 	// Open Loop Buck PWM Driver
//----------------------------------------------------------------------
	// Lib Module connection to "nets"
	//----------------------------------------
	// Connect the PWM Driver input to an input variable, Open Loop System
	#if (BUCK_PWM_NO == 1)
		PWMDRV_1ch_Duty1 = &Duty1A;
	#elif (BUCK_PWM_NO == 2)
		PWMDRV_1ch_Duty2 = &Duty1A;
	#elif (BUCK_PWM_NO == 3)
		PWMDRV_1ch_Duty3 = &Duty1A;
	#elif (BUCK_PWM_NO == 4)
		PWMDRV_1ch_Duty4 = &Duty1A;
	#elif (BUCK_PWM_NO == 5)
		PWMDRV_1ch_Duty5 = &Duty1A;
	#endif

// ADC feedback connection
	ADCDRV_1ch_Rlt1 = &Adc_Vout1;

// Initialize the net variable
	Duty1A =_IQ24(0.0);

#endif // (INCR_BUILD == 1)

//----------------------------------------------------------------------
#if (INCR_BUILD == 2) 	// Closed Voltage Loop with SFRA
//----------------------------------------------------------------------
	// Lib Module connection to "nets"
	//----------------------------------------
	// Connect the PWM Driver input to an input variable
	#if (BUCK_PWM_NO == 1)
		PWMDRV_1ch_Duty1 = &Duty1A;
	#elif (BUCK_PWM_NO == 2)
		PWMDRV_1ch_Duty2 = &Duty1A;
	#elif (BUCK_PWM_NO == 3)
		PWMDRV_1ch_Duty3 = &Duty1A;
	#elif (BUCK_PWM_NO == 4)
		PWMDRV_1ch_Duty4 = &Duty1A;
	#elif (BUCK_PWM_NO == 5)
		PWMDRV_1ch_Duty5 = &Duty1A;
	#endif

// ADC feedback connection
	ADCDRV_1ch_Rlt1 = &Adc_Vout1;

//2P2Z connections for the Voltage Loop
	CNTL_2P2Z_Ref1 = &Vout_Ref_wInj;				// Slewed Voltage command
	CNTL_2P2Z_Out1 = &Duty1A;						// Duty command
	CNTL_2P2Z_Fdbk1 = &Adc_Vout1;					// O/P Voltage feedback
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;	// point to first coeff.

	// Coefficients for the Voltage Loop
	#if (ACTIVE_COMP == 1)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_1);                	// B2
		CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_1);  				// B1
		CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_1);      			// B0
		CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_1);                 // A2
		CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_1);                 // A1
		CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_1;					  	//Clamp Hi
		CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_1; 					  	//Clamp Min
		CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_1; 					//Clamp IMin
	#elif (ACTIVE_COMP == 2)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_2);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_2);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_2);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_2);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_2);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_2;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_2; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_2;
	#elif (ACTIVE_COMP == 3)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_3);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_3);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_3);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_3);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_3);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_3;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_3; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_3;
	#elif (ACTIVE_COMP == 4)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_4);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_4);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_4);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_4);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_4);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_4;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_4; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_4;
	#elif (ACTIVE_COMP == 5)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_5);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_5);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_5);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_5);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_5);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_5;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_5; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_5;
	#endif

	// DLOG block connections
	// Store address of the system variables that need to be logged
	DLOG_1ch_Input1 =&Vout1_dlog;
	// Point the BuffPtr to the buffer location
	DLOG_1ch_OutputBuff1 =DBUFF1;
	// Setup Size, Trigger Value and Pre Scalar
	DLOG_1ch_TrigVal1 = _IQ(0.1);
	DLOG_1ch_PreScalar1 = 15;
	DLOG_1ch_Size1=DLOG_SIZE;
	// Zero the buffers
	DLOG_BuffInit(DBUFF1, DLOG_SIZE);

// Initialize the net variable
	Duty1A =_IQ24(0.0);

#endif // (INCR_BUILD == 2)


#if (INCR_BUILD == 3) 	// Closed Voltage Loop without SFRA
//----------------------------------------------------------------------
	// Lib Module connection to "nets"
	//----------------------------------------
	// Connect the PWM Driver input to an input variable
	#if (BUCK_PWM_NO == 1)
		PWMDRV_1ch_Duty1 = &Duty1A;
	#elif (BUCK_PWM_NO == 2)
		PWMDRV_1ch_Duty2 = &Duty1A;
	#elif (BUCK_PWM_NO == 3)
		PWMDRV_1ch_Duty3 = &Duty1A;
	#elif (BUCK_PWM_NO == 4)
		PWMDRV_1ch_Duty4 = &Duty1A;
	#elif (BUCK_PWM_NO == 5)
		PWMDRV_1ch_Duty5 = &Duty1A;
	#endif

// ADC feedback connection
	ADCDRV_1ch_Rlt1 = &Adc_Vout1;

//2P2Z connections for the Voltage Loop
	CNTL_2P2Z_Ref1 = &Vout1SetSlewed;				// Slewed Voltage command
	CNTL_2P2Z_Out1 = &Duty1A;						// Duty command
	CNTL_2P2Z_Fdbk1 = &Adc_Vout1;					// O/P Voltage feedback
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;	// point to first coeff.

// Coefficients for the Voltage Loop
	// Coefficients for the Voltage Loop
	#if (ACTIVE_COMP == 1)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_1);                	// B2
	    CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_1);  				// B1
	    CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_1);      			// B0
	    CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_1);                 // A2
	    CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_1);                 // A1
	    CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_1;					  	//Clamp Hi
	    CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_1; 					  	//Clamp Min
	    CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_1; 					//Clamp IMin
	#elif (ACTIVE_COMP == 2)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_2);                	// B2
		CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_2);  				// B1
		CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_2);      			// B0
		CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_2);                 // A2
		CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_2);                 // A1
		CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_2;					  	//Clamp Hi
		CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_2; 					  	//Clamp Min
		CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_2;
	#elif (ACTIVE_COMP == 3)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_3);                	// B2
		CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_3);  				// B1
		CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_3);      			// B0
		CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_3);                 // A2
		CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_3);                 // A1
		CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_3;					  	//Clamp Hi
		CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_3; 					  	//Clamp Min
		CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_3;
	#elif (ACTIVE_COMP == 4)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_4);                	// B2
		CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_4);  				// B1
		CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_4);      			// B0
		CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_4);                 // A2
		CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_4);                 // A1
		CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_4;					  	//Clamp Hi
		CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_4; 					  	//Clamp Min
		CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_4;
	#elif (ACTIVE_COMP == 5)
	// Coefficient init	--- Coeeficient values in Q26
		CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_5);                	// B2
		CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_5);  				// B1
		CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_5);      			// B0
		CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_5);                 // A2
		CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_5);                 // A1
		CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_5;					  	//Clamp Hi
		CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_5; 					  	//Clamp Min
		CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_5;
	#endif

	// DLOG block connections
	// Store address of the system variables that need to be logged
	DLOG_1ch_Input1 =&Vout1_dlog;
	// Point the BuffPtr to the buffer location
	DLOG_1ch_OutputBuff1 =DBUFF1;
	// Setup Size, Trigger Value and Pre Scalar
	DLOG_1ch_TrigVal1 = _IQ(0.1);
	DLOG_1ch_PreScalar1 = 15;
	DLOG_1ch_Size1=DLOG_SIZE;
	// Zero the buffers
	DLOG_BuffInit(DBUFF1, DLOG_SIZE);

// Initialize the net variable
	Duty1A =_IQ24(0.0);

#endif // (INCR_BUILD == 3)

//----------------------------------------------------------------------
// Configure selected PWM, Comparator and DAC for over current protection
	EALLOW;	
// Define an event (DCAEVT1) based on Comparator Output
	ePWM[BUCK_PWM_NO]->DCTRIPSEL.bit.DCAHCOMPSEL = (ADC_IL_COMPARATOR+7); 	// DCAH = Comparator output
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

	DacDrvCnf(ADC_IL_COMPARATOR, Itrip1, 0, 2, 0);		// Selected comparator, DACval = Itrip1, DAC Source is DACval, Ramp Source = don't care, Slope = don't care

//===========================================================================
//Active load PWM drive configuration - EPWM6 is used
//===========================================================================
	//Time Base SubModule Register
	EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;		// set Immediate load
	EPwm6Regs.TBPRD = Active_load_PRD;
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;		// divide by 4
	EPwm6Regs.TBCTL.bit.CLKDIV = 3;					// divide by 8
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; 		// sync "down-stream"

	// Counter compare submodule registers
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	// Action Qualifier SubModule Registers
	EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;

  	// Configure SOC event generation at PWM6 level - for slower ADC conversions
	EPwm6Regs.ETSEL.bit.SOCAEN  =  1;
	EPwm6Regs.ETSEL.bit.SOCASEL =  ET_CTR_ZERO; // Use CTR = ZRO events as trigger
    EPwm6Regs.ETPS.bit.SOCAPRD = 1; 	        // Generate pulse on 1st event

	EPwm6Regs.CMPA.half.CMPA = 0;				// Active load disabled initially

//===========================================================================
//	SFRA Initialization
//===========================================================================
	//SFRA Object Initialization
	//Specify the injection amplitude
	SFRA1.amplitude=_IQ26(0.005);
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

	SFRA_IQ_INIT(&SFRA1);

//====================================================================================
// Start all enabled ePWM module clocks

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; 
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
   	ePWM[BUCK_PWM_NO]->ETSEL.bit.INTSEL = ET_CTR_PRD;  	// INT on PRD event
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
				SFRA_IQ_INIT(&SFRA1);
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
		Duty1A = SFRA_IQ_INJECT(Duty1ASetSlewed);		//Duty1ASetSlewed - Slewed Duty command (Duty1A_Set - set buy user)
	#elif INCR_BUILD == 2
		Vout_Ref_wInj = SFRA_IQ_INJECT(Vout1SetSlewed);
	#endif

	DPL_Func();

	#if INCR_BUILD != 3
		SFRA_IQ_COLLECT(&Duty1A,&Adc_Vout1);
	#endif
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
//--------------------------------------------------------
void A1(void) 
//--------------------------------------------------------
{
	Itrip1 = ( (long) Gui_ItripSet * (long) iK_IL1) >> 14;	 			// (Q15 * Q14) >> 14 = Q15
	EALLOW;
		Comp[ADC_IL_COMPARATOR]->DACVAL.bit.DACVAL = ((Itrip1)>>5);		// DAC Value is in Q10
	EDIS;

	Pgain1 	= Pgain1_Gui*67108;		// Q26
	Igain1 	= Igain1_Gui*67108;		// Q26
	Dgain1 	= Dgain1_Gui*67108;		// Q26 

  if (coeff_change == 1)
   {
		No_2p2z = 1;	// Used to disable 2P2Z execution when coefficients are being changed

	if (pid2p2z_Gui == 0)
		{
		// Voltage loop coefficient update
		CNTL_2P2Z_CoefStruct1.b2   = Dgain1;                            	// B2
    	CNTL_2P2Z_CoefStruct1.b1   = (Igain1 - Pgain1 - Dgain1 - Dgain1);  	// B1
   	 	CNTL_2P2Z_CoefStruct1.b0   = (Pgain1 + Igain1 + Dgain1);      		// B0
    	CNTL_2P2Z_CoefStruct1.a2   = 0.0;                              		// A2
    	CNTL_2P2Z_CoefStruct1.a1   = _IQ26(1.0);                       		// A1
		}
	else
		{
		// Coefficients for the Voltage Loop
		#if (ACTIVE_COMP == 1)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_1);                	// B2
		    CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_1);  				// B1
		    CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_1);      			// B0
		    CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_1);                 // A2
		    CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_1);                 // A1
		    CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_1;					  	//Clamp Hi
		    CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_1; 					  	//Clamp Min
		    CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_1; 					//Clamp IMin
		#elif (ACTIVE_COMP == 2)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_2);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_2);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_2);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_2);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_2);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_2;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_2; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_2;
		#elif (ACTIVE_COMP == 3)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_3);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_3);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_3);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_3);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_3);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_3;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_3; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_3;
		#elif (ACTIVE_COMP == 4)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_4);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_4);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_4);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_4);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_4);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_4;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_4; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_4;
		#elif (ACTIVE_COMP == 5)
		// Coefficient init	--- Coeeficient values in Q26
			CNTL_2P2Z_CoefStruct1.b2   = _IQ26(CNTL_3p3z_B2_5);                	// B2
			CNTL_2P2Z_CoefStruct1.b1   = _IQ26(CNTL_3p3z_B1_5);  				// B1
			CNTL_2P2Z_CoefStruct1.b0   = _IQ26(CNTL_3p3z_B0_5);      			// B0
			CNTL_2P2Z_CoefStruct1.a2   = _IQ26(CNTL_3p3z_A2_5);                 // A2
			CNTL_2P2Z_CoefStruct1.a1   = _IQ26(CNTL_3p3z_A1_5);                 // A1
			CNTL_2P2Z_CoefStruct1.max  = CNTL_3p3z_Max_5;					  	//Clamp Hi
			CNTL_2P2Z_CoefStruct1.min  = CNTL_3p3z_Min_5; 					  	//Clamp Min
			CNTL_2P2Z_CoefStruct1.i_min  = CNTL_3p3z_IMin_5;
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
void A2(void)   // Slew Rate
//-----------------------------------------------------------------
{	 
	// This is an example code for implementing the slew rate control in
	// a slower state machine
	// Duty1ASlewRate should be set as a positive value
#if INCR_BUILD == 1

	if (Duty1A_Set > 7549747)							// Clamp Duty command to 0.45 (Q24)
		Duty1A_Set = 7549747;

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
			EPwm6Regs.CMPA.half.CMPA = Active_load_PRD;
			Gui_dlog_trig1 = Gui_Vout1;
		}
		else
		{
			EPwm6Regs.CMPA.half.CMPA = Active_load_PRD>>1;
			Gui_dlog_trig1 = Gui_Vout1 + 205;		//Vout + 0.1V = 0.1*2^12
		}
	}
	else
	{
			EPwm6Regs.CMPA.half.CMPA = 0;

			Gui_dlog_trig1 = Gui_Vout1;
	}			

	if (Gui_Vout1 < 410)			// <0.1V
			Gui_dlog_trig1 = 245;	// 0.06V
	
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
void B1(void)
//----------------------------------------
{
// Please refer to DP_BoosterPack_Calculations.xls
// Voltage measurement calculated by:
// Gui_Vin = VinAvg * K_Vin, where VinAvg = sum of 8 VinR samples
// Gui_Vout1 = Vou1tAvg * K_Vout1, where Vout1Avg = sum of 8 Vout1R samples
 
	HistPtr++;
	if (HistPtr >= 8)	HistPtr = 0;

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------
	Hist_Vin[HistPtr] = VinR; 			// Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vin[i]; // Q12 * 8 = Q15
	Gui_Vin = ( (long) temp_Scratch * (long) K_Vin ) >> 15; // (Q15 * Q15)>>15 = Q15

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------   
	Hist_Vout1[HistPtr] = Vout1R; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vout1[i]; // Q12 * 8 = Q15
	Gui_Vout1 = ( (long) temp_Scratch * (long) K_Vout1 ) >> 15; // (Q15 * Q15)>>15 = Q15

// Current measurement calculated by:
//	Gui_IL1 = IL1Avg * K_Ifb, where IL1Avg = sum of 8 IL1R samples

//BoxCar Averages - Input Raw samples into History arrays 
//----------------------------------------------------------------
	Hist_IL1[HistPtr] = IL1R; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_IL1[i]; // Q12 * 8 = Q15
	Gui_IL1 = ( (long) temp_Scratch * (long) K_IL1 ) >> 15; // (Q15 * Q15)>>15 = Q15

// Voltage setting calculated by:
// Vref1 = Gui_VSet1 * iK_Vout1, where iK_Vout1 = 1/K_Vout1 (i.e. inverse K_Vout1)
// view and set following variable in Watch Window as:
//	Gui_VSet1 = Q12 (Used as Q15 below)
	if (Gui_VSet1 < 24576)				// Command voltage < 6V
		Vref1 = ( (long) Gui_VSet1 * (long) iK_Vout1) >> 5; // (Q15 * Q14) >> 5 = Q24
//	else - If voltage command is > 6V, maintain the old output voltage value

	#if	INCR_BUILD != 1
		DLOG_1ch_TrigVal1 = ( (long) Gui_dlog_trig1)<<9;	//Q24 = Q15<<9
	#endif

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
			GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	//turn on/off LED D9 on the F28069M LaunchPad
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
		SFRA_IQ_BACKGROUND(&SFRA1);
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
	if (ClearFault1 == 1)
	{
		EALLOW;
		(*ePWM[BUCK_PWM_NO]).TZCLR.bit.OST = 1;
		EDIS;		
		ClearFault1 = 0;
	}

	if ( (*ePWM[BUCK_PWM_NO]).TZFLG.bit.OST == 1 )
		FaultFlg1 = 1; 
	else FaultFlg1 = 0;  
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------

}
//----------------------------------------
void C2(void) // Code to check switch position - OL Excitation
//----------------------------------------
{
	#if	(INCR_BUILD != 1)
	  if (Start_Flag == 1)				// Execute only till the first time switch is turned ON
		{
		  CNTL_2P2Z_Out1 = &temp;
		  Duty1A = _IQ24(0.05);			// OL excitation
		}
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
		  if (Gui_Vout1 > 100) 			// Output voltage > 0.025V - Switch SW1 is in ON position
		  {
			  CNTL_2P2Z_Out1 = &Duty1A;
			  if (OUTPUT_VOLTAGE < 6)	// Max output voltage restricted to 6V
			  	  Gui_VSet1 = OUTPUT_VOLTAGE*4096;
			  else	Gui_VSet1 = 0;
			  Start_Flag = 0;
		  }
		  Duty1A = _IQ24(0.0);			// Turn OFF excitation
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


