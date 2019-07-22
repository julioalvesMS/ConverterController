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
--diag_suppress=16002
MEMORY
{
PAGE 0 :   /* Program Memory */
          /* Memory (RAM/FLASH) blocks can be moved to PAGE1 for data allocation */
          /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002
   RAMM0           	: origin = 0x000122, length = 0x0002DE
   RAMD0           	: origin = 0x00B000, length = 0x000800
   RAMLS0          	: origin = 0x008000, length = 0x000800
   RAMLS1          	: origin = 0x008800, length = 0x000800
   RAMLS2      		: origin = 0x009000, length = 0x000800
//   RAMLS3      		: origin = 0x009800, length = 0x000800
   RAMLS4      		: origin = 0x00A000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
//   FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASHI           : origin = 0x0A9E20, length = 0x0061E0	/* on-chip Flash */
   FLASHJ           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
   FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASHN           : origin = 0x0BE000, length = 0x002000	/* on-chip Flash */

   IQTABLES    		: origin = 0x0A8000, length = 0x000B50    /* IQ Math Tables in part of FLASHI */
   IQTABLES2   		: origin = 0x0A8B50, length = 0x00008C    /* IQ Math Tables in part of FLASHI */
   IQTABLES3   		: origin = 0x0A8BDC, length = 0x0000AA	  /* IQ Math Tables in part of FLASHI */
   IQMATH      		: origin = 0x0A8C86, length = 0x000afa	  /* IQ Math functions in part of FLASHI */
   FPUTABLES 		: origin = 0x0A9780, length = 0x0006A0

PAGE 1 : /* Data Memory */
         /* Memory (RAM/FLASH) blocks can be moved to PAGE0 for program allocation */

   BOOT_RSVD       : origin = 0x000002, length = 0x000120     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAMD1           : origin = 0x00B800, length = 0x000800

   RAMLS5      : origin = 0x00A800, length = 0x000800

   RAMGS0      : origin = 0x00C000, length = 0x001000
   RAMGS1      : origin = 0x00D000, length = 0x001000
   RAMGS2      : origin = 0x00E000, length = 0x001000
   RAMGS3      : origin = 0x00F000, length = 0x001000
   RAMGS4      : origin = 0x010000, length = 0x001000
   RAMGS5      : origin = 0x011000, length = 0x001000
   RAMGS6      : origin = 0x012000, length = 0x001000
   RAMGS7      : origin = 0x013000, length = 0x001000
   RAMGS8      : origin = 0x014000, length = 0x001000
   RAMGS9      : origin = 0x015000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000
   RAMGS11     : origin = 0x017000, length = 0x001000

   dataRAM     : origin = 0x009800, length = 0x000800		 /* on-chip RAM block LS3 */

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
   .cinit            : > FLASHB,     PAGE = 0
   .pinit            : > FLASHB,     PAGE = 0
   .text             : > FLASHB | FLASHC	    PAGE = 0

   codestart         : > BEGIN       PAGE = 0

   ramfuncs          : LOAD = FLASHD,
                       RUN = RAMLS0 | RAMLS1 | RAMLS2,
                       LOAD_START(_RamfuncsLoadStart),
                       LOAD_SIZE(_RamfuncsLoadSize),
                       LOAD_END(_RamfuncsLoadEnd),
                       RUN_START(_RamfuncsRunStart),
                       RUN_SIZE(_RamfuncsRunSize),
                       RUN_END(_RamfuncsRunEnd),
                       PAGE = 0
    {
      					--library=SFRA_F_Lib.lib<SFRA_F_INJECT.obj>
   						--library=SFRA_F_Lib.lib<SFRA_F_COLLECT.obj>
	}


   /* Allocate uninitalized data sections: */
  .stack              : > RAMM1        PAGE = 1
   .ebss               : >> RAMLS5 | RAMGS0 | RAMGS1       PAGE = 1
   .esysmem            : > RAMLS5       PAGE = 1

   /* Initalized sections go in Flash */
  .econst             : >> FLASHF | FLASHG | FLASHH      PAGE = 0
   .switch             : > FLASHB      PAGE = 0

   .reset              : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   IQmath           : > FLASHA,     PAGE = 0
   IQmathTables     : > IQTABLES,  PAGE = 0
   FPUmathTables	: > FPUTABLES, 	PAGE =0

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


