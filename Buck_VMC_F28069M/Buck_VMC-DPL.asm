;----------------------------------------------------------------------------------
;	FILE:			Buck_VMC-DPL.asm
;
;	Description:	Buck_VMC-DPL.asm contains the time critical 'control type' code for the system.
;					It also contains the initailization routine for all the macros 
;					being used in the system
;   
;   Revision/ Version: See Buck_VMC-Main.c
;----------------------------------------------------------------------------------

		;Gives peripheral addresses visibility in assembly
	    .cdecls   C,LIST,"PeripheralHeaderIncludes.h"

		;include C header file - sets INCR_BUILD etc.(used in conditional builds)
		.cdecls C,NOLIST, "Buck_VMC-Settings.h"

		;Include files for the Power Library Maco's being used by the system
		.include "PWMDRV_1ch.asm" 
		.include "DACDRV_RAMP.asm"
		.include "ADCDRV_1ch.asm"
		.include "CNTL_2P2Z.asm" 	
		.include "DLOG_1ch.asm"
;=============================================================================
; Digital Power library - Initailization Routine 
;=============================================================================
		; label to DP initialization function
		.def _DPL_Init	
		
		; dummy variable for pointer initialization
ZeroNet	  .usect "ZeroNet_Section",2,1,1	; output terminal 1
temp_val1 .usect "ZeroNet_Section",2,1,1
temp_val2 .usect "ZeroNet_Section",2,1,1

		.text
_DPL_Init:
		;context save - push any unprotected registers onto stack
		PUSH  	XAR1
		PUSH  	XAR2
		PUSH  	XAR3

		ZAPA
		MOVL	XAR0, #ZeroNet
		MOVL	*XAR0, ACC

		; Initialize all the DP library macro used here 
		;---------------------------------------------------------
		.if(INCR_BUILD = 1)
			ADCDRV_1ch_INIT 		1	; Vout1
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch_INIT 	1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch_INIT 	2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch_INIT 	3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch_INIT 	4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch_INIT 	5	; PWM5
			.endif
		.endif
		;---------------------------------------------------------			

		;---------------------------------------------------------
		.if(INCR_BUILD = 2)
			ADCDRV_1ch_INIT 		1	; Vout1
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch_INIT 	1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch_INIT 	2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch_INIT 	3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch_INIT 	4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch_INIT 	5	; PWM5
			.endif
			CNTL_2P2Z_INIT			1
			DLOG_1ch_INIT 			1	; DLOG_1CH Initialization
		.endif
		;---------------------------------------------------------

		;---------------------------------------------------------
		.if(INCR_BUILD = 3)
			ADCDRV_1ch_INIT 		1	; Vout1
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch_INIT 	1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch_INIT 	2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch_INIT 	3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch_INIT 	4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch_INIT 	5	; PWM5
			.endif
			CNTL_2P2Z_INIT			1
			DLOG_1ch_INIT 			1	; DLOG_1CH Initialization
		.endif
		;---------------------------------------------------------

		;context save - pop registers from stack
		POP   	XAR3
		POP   	XAR2
		POP   	XAR1
		LRETR

;-----------------------------------------------------------------------------------------
; Digital power library based run time function
;-----------------------------------------------------------------------------------------
		; label to DP DPL_Func Run function
		.def	_DPL_Func

		.sect "ramfuncs"

; Digital power library based time critical control code
;-----------------------------------------------------------------------------------------
_DPL_Func:
		;context save - push any unprotected registers onto stack
		PUSH  	XAR1
		PUSH  	XAR2
		PUSH  	XAR3

;-----------------------------------------------------------------------------------------
; call DP library modules
;---------------------------------------------------------
		.if(INCR_BUILD = 1)
			ADCDRV_1ch		1	; Read ADC result - Vout1
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch 		1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch 		2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch 		3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch 		4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch 		5	; PWM5
			.endif
		.endif
		;----------------------------------------------------------

		;---------------------------------------------------------
		.if(INCR_BUILD = 2)
			ADCDRV_1ch			1	; Read ADC result - Vout1
			.ref	_No_2p2z
			MOVW	DP, #(_No_2p2z)
			MOV		AL, @(_No_2p2z)
			CMPB 	AL,#0x1
			B		SKIP1_2P2Z, EQ	; If equal - coefficients are being changed in the slower loop --> don't execute 2P2Z

			CNTL_2P2Z			1	; Voltage Controller
SKIP1_2P2Z:
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch 		1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch 		2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch 		3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch 		4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch 		5	; PWM5
			.endif

			.ref	_Adc_Vout1
			MOVW	DP, #(_Adc_Vout1)
			MOVL	ACC, @(_Adc_Vout1)
			LSL		ACC, #7
			MOVW	DP,#(temp_val1)
			MOV		@temp_val1, AH
			MOV		T, @temp_val1
			.ref	_K_Vout1
			MOVW	DP, #(_K_Vout1)
			MPY		ACC,T,@(_K_Vout1); Q15*Q15 = Q30
			CLRC	SXM
			SFR		ACC, #6			 ; Q30 to Q24
			.ref	_Vout1_dlog
			MOVW	DP, #(_Vout1_dlog)
			MOVL	@(_Vout1_dlog), ACC

			DLOG_1ch 		1
			
		.endif
		;----------------------------------------------------------

		;---------------------------------------------------------
		.if(INCR_BUILD = 3)
			ADCDRV_1ch			1	; Read ADC result - Vout1
			.ref	_No_2p2z
			MOVW	DP, #(_No_2p2z)
			MOV		AL, @(_No_2p2z)
			CMPB 	AL,#0x1
			B		SKIP1_2P2Z, EQ	; If equal - coefficients are being changed in the slower loop --> don't execute 2P2Z

			CNTL_2P2Z			1	; Voltage Controller
SKIP1_2P2Z:
			.if(BUCK_PWM_NO = 1)
				PWMDRV_1ch 		1	; PWM1
			.endif
			.if(BUCK_PWM_NO = 2)
				PWMDRV_1ch 		2	; PWM2
			.endif
			.if(BUCK_PWM_NO = 3)
				PWMDRV_1ch 		3	; PWM3
			.endif
			.if(BUCK_PWM_NO = 4)
				PWMDRV_1ch 		4	; PWM4
			.endif
			.if(BUCK_PWM_NO = 5)
				PWMDRV_1ch 		5	; PWM5
			.endif

			.ref	_Adc_Vout1
			MOVW	DP, #(_Adc_Vout1)
			MOVL	ACC, @(_Adc_Vout1)
			LSL		ACC, #7
			MOVW	DP,#(temp_val1)
			MOV		@temp_val1, AH
			MOV		T, @temp_val1
			.ref	_K_Vout1
			MOVW	DP, #(_K_Vout1)
			MPY		ACC,T,@(_K_Vout1); Q15*Q15 = Q30
			CLRC	SXM
			SFR		ACC, #6			 ; Q30 to Q24
			.ref	_Vout1_dlog
			MOVW	DP, #(_Vout1_dlog)
			MOVL	@(_Vout1_dlog), ACC

			DLOG_1ch 		1

		.endif
		;----------------------------------------------------------

;-----------------------------------------------------------------------------------------
; Interrupt management before exit
	.if(EPWMn_DPL_ISR=1)
	; Case where ISR is triggered by PWM
		.if(BUCK_PWM_NO = 1)
			MOVW 	DP,#_EPwm1Regs.ETCLR
			MOV 	@_EPwm1Regs.ETCLR,#0x01			; Clear EPWM1 Int flag
		.endif

		.if(BUCK_PWM_NO = 2)
			MOVW 	DP,#_EPwm2Regs.ETCLR
			MOV 	@_EPwm2Regs.ETCLR,#0x01			; Clear EPWM2 Int flag
		.endif

		.if(BUCK_PWM_NO = 3)
			MOVW 	DP,#_EPwm3Regs.ETCLR
			MOV 	@_EPwm3Regs.ETCLR,#0x01			; Clear EPWM3 Int flag
		.endif

		.if(BUCK_PWM_NO = 4)
			MOVW 	DP,#_EPwm4Regs.ETCLR
			MOV 	@_EPwm4Regs.ETCLR,#0x01			; Clear EPWM4 Int flag
		.endif

		.if(BUCK_PWM_NO = 5)
			MOVW 	DP,#_EPwm5Regs.ETCLR
			MOV 	@_EPwm5Regs.ETCLR,#0x01			; Clear EPWM5 Int flag
		.endif

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 3
		MOV 	@_PieCtrlRegs.PIEACK, #0x4

	.endif 

	.if(CLAn_DPL_ISR=1)
	; Case where ISR is triggered by CLA task n 
		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 11
		MOV 	@_PieCtrlRegs.PIEACK, #0x0400	
		
	.endif 
	
	.if(ADC_DPL_ISR=1)
	; Case where ISR is triggered by ADC 
		MOVW 	DP,#_AdcRegs.ADCINTFLGCLR
		MOV 	@_AdcRegs.ADCINTFLGCLR,#0x01		; Clear ADCINT1 Flag

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 1
		MOV 	@_PieCtrlRegs.PIEACK,#0x1
	.endif 

		;context save - pop registers from stack
		POP   	XAR3
		POP   	XAR2
		POP   	XAR1
		LRETR							; return from function

; end of file

