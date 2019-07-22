/*==================================================================================*/
/*	User specific Linker command file for running from FLASH						*/
/*==================================================================================*/
/*	FILE:			F28069_FLASH_DP_BoosterPack.CMD                                    */
/*                                                                                  */
/*	Description:	Linker command file for User custom sections targetted to run   */
/*					from FLASH.  			                                        */
/*                                                                                  */
/*  Target:  		TMS320F28069         					                        */
/*                                                                                  */
/*	Version: 		1.00                                							*/
/*                                                                                  */
/*----------------------------------------------------------------------------------*/
/*  Copyright Texas Instruments © 2015                                			    */
/*----------------------------------------------------------------------------------*/
/*  Revision History:                                                               */
/*----------------------------------------------------------------------------------*/
/*  Date	  | Description                                                         */
/*----------------------------------------------------------------------------------*/
/*  01/20/15  | Release 1.0  		New release.                                    */
/*----------------------------------------------------------------------------------*/
/* Define the memory block start/length for the F2806x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F28069 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/
MEMORY
{
PAGE 0 :
  /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN           	: origin = 0x000000, length = 0x000002
   RAMM0           	: origin = 0x000122, length = 0x0002DE
   RAMD0           	: origin = 0x00B000, length = 0x000800
   RAMLS0          	: origin = 0x008000, length = 0x000800
   RAMLS1_LS2       : origin = 0x008800, length = 0x001000
//   RAMLS3      		: origin = 0x009800, length = 0x000800
   RAMLS4      		: origin = 0x00A000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x000120     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAMD1           : origin = 0x00B800, length = 0x000800

   RAMLS5      : origin = 0x00A800, length = 0x000800

   RAMGS0      : origin = 0x00C000, length = 0x001000
   RAMGS1      : origin = 0x00D000, length = 0x001000
//   RAMGS2      : origin = 0x00E000, length = 0x001000
   RAMGS3      : origin = 0x00FE20, length = 0x0001E0
   RAMGS4      : origin = 0x010000, length = 0x001000
   RAMGS5      : origin = 0x011000, length = 0x001000
   RAMGS6      : origin = 0x012000, length = 0x001000
   RAMGS7      : origin = 0x013000, length = 0x001000
   RAMGS8      : origin = 0x014000, length = 0x001000
   RAMGS9      : origin = 0x015000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000
   RAMGS11     : origin = 0x017000, length = 0x001000
   RAMGS12     : origin = 0x018000, length = 0x001000
   RAMGS13     : origin = 0x019000, length = 0x001000
   RAMGS14     : origin = 0x01A000, length = 0x001000
   RAMGS15     : origin = 0x01B000, length = 0x001000

   dataRAM     : origin = 0x009800, length = 0x000800		 /* on-chip RAM block LS3 */

   IQTABLES    : origin = 0x00E000, length = 0x000B50    /* IQ Math Tables in part of RAMGS2 */
   IQTABLES2   : origin = 0x00EB50, length = 0x00008C    /* IQ Math Tables in part of RAMGS2 */
   IQTABLES3   : origin = 0x00EBDC, length = 0x0000AA	 /* IQ Math Tables in part of RAMGS2 */
   IQMATH      : origin = 0x00EC86, length = 0x000afa	 /* IQ Math functions in part of RAMGS2 and RAMGS3 */
   FPUTABLES   : origin = 0x00F780, length = 0x0006A0	 /* FPU Tables in part of RAMGS2 and RAMGS3 */


}

/* Allocate sections to memory blocks.
   Note:
         codestart user defined section in DSP28_CodeStartBranch.asm used to redirect code
                   execution when booting to flash
         ramfuncs  user defined section to store functions that will be copied from Flash into RAM
*/

SECTIONS
{
   /* Allocate program areas: */
   .cinit           : > RAMM0,     PAGE = 0
   .pinit           : > RAMM0,     PAGE = 0
   .text            : >>RAMM0 | RAMD0 |  RAMLS0 | RAMLS1_LS2,   PAGE = 0

   codestart        : > BEGIN       PAGE = 0

   ramfuncs         : >> RAMLS1_LS2 | RAMLS4,   PAGE = 0
   {
      					--library=SFRA_F_Lib.lib<SFRA_F_INJECT.obj>
   						--library=SFRA_F_Lib.lib<SFRA_F_COLLECT.obj>
	}


   /* Allocate uninitalized data sections: */
   .stack           : > RAMM1        PAGE = 1
   .ebss            : >> RAMLS5 | RAMGS0 | RAMGS1       PAGE = 1
   .esysmem         : > RAMLS5       PAGE = 1

   /* Initalized sections go in Flash */
   .econst          : > RAMLS5,    PAGE = 1
   .switch          : > RAMM0,     PAGE = 0

   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   IQmath           : > IQMATH,     PAGE = 1
   IQmathTables     : > IQTABLES,   PAGE = 1
   FPUmathTables	: > FPUTABLES, 	PAGE = 1

  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
/*
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }*/

   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
/*
   IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }*/


}
    

SECTIONS
{
	/*************       DPLIB Sections C28x      ************************/
	/* ADCADRV_1ch section */
	ADCADRV_1ch_Section				: > dataRAM				PAGE = 1

	/* ADCDRV_4ch section */
	ADCDRV_4ch_Section				: > dataRAM				PAGE = 1

	/* CNTL_2P2Z section */
	CNTL_2P2Z_Section				: > dataRAM				PAGE = 1
	CNTL_2P2Z_InternalData			: > dataRAM				PAGE = 1
	CNTL_2P2Z_Coef					: > dataRAM				PAGE = 1

	/* CNTL_3P3Z section */
	CNTL_3P3Z_Section				: > dataRAM				PAGE = 1
	CNTL_3P3Z_InternalData			: > dataRAM				PAGE = 1
	CNTL_3P3Z_Coef					: > dataRAM				PAGE = 1

	/* DACDRV_RAMP section */
	DACDRV_RAMP_Section				: > dataRAM				PAGE = 1

	/*DLOG_1CH section */
	DLOG_1ch_Section				: > dataRAM				PAGE = 1
	DLOG_BUFF						: > dataRAM				PAGE = 1

	/*MATH_EMAVG section */
	MATH_EMAVG_Section				: > dataRAM				PAGE = 1

	/*PFC_ICMD section*/
	PFC_ICMD_Section				: > dataRAM				PAGE = 1

	/*PFC_INVSQR section*/
	PFC_INVSQR_Section				: > dataRAM				PAGE = 1

	/* PWMDRV_1ch driver section */
	PWMDRV_1ch_Section				: > dataRAM				PAGE = 1

	/* PWMDRV_1chHiRes driver section */
	PWMDRV_1chHiRes_Section			: > dataRAM				PAGE = 1

	/* PWMDRV_PFC2PhiL driver section */
	PWMDRV_PFC2PhiL_Section			: > dataRAM				PAGE = 1

 	/* PWMDRV_PSFB driver section */
	PWMDRV_PSFB_Section				: > dataRAM				PAGE = 1

	/* PWMDRV_DualUpDwnCnt driver section */
	PWMDRV_DualUpDwnCnt_Section		: > dataRAM				PAGE = 1

	/* PWMDRV_ComplPairDB driver section */
	PWMDRV_ComplPairDB_Section		: > dataRAM				PAGE = 1

	/* ZeroNet_Section  */
	ZeroNet_Section					: > dataRAM				PAGE = 1
	
	/* SFRA Data  */
	SFRA_F_Data					: > dataRAM, ALIGN = 64, PAGE = 1

}


