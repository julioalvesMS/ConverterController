#ifndef DEFINESCONTROLCARDV2_H_
#define DEFINESCONTROLCARDV2_H_

//###########################################################################
//
// Cabeçalho:
// Título:  Config_Gpio.DFIG.c

//Descrição :
//
///###########################################################################
// Author: TÁRCIO ANDRÉ 12/12/2017
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

//#######################################################DEFINIÇÕES DE NOMES PARA AÇOES###############################

// RESET E BKR DOS VSC
#define LIGARESETVSC	    GpioDataRegs.GPBSET.bit.GPIO63=1;              // RESETA VSC
#define DESLIGARESETVSC	    GpioDataRegs.GPBCLEAR.bit.GPIO63=1;            // DESLIGA VSC

#define LIGARESETVSC2	    GpioDataRegs.GPCSET.bit.GPIO86=1;              // RESETA VSC2
#define DESLIGARESETVSC2	GpioDataRegs.GPCCLEAR.bit.GPIO86=1;            // DESLIGA VSC2

#define LIGABKR       	    GpioDataRegs.GPBSET.bit.GPIO59=1;          	   // RESETA BKR
#define DESLIGABKR       	GpioDataRegs.GPBCLEAR.bit.GPIO59=1;            // DESLIGA BKR

#define LIGABKR2	        GpioDataRegs.GPBSET.bit.GPIO58=1;              // RESETA BKR2
#define DESLIGABKR2	    	GpioDataRegs.GPBCLEAR.bit.GPIO58=1;            // DESLIGA BKR2

// RESET DO TZ
#define DESLISGATZSENSORES GpioDataRegs.GPBSET.bit.GPIO34=1;   			   // RESETA BKR
#define LISGATZSENSORES    GpioDataRegs.GPBCLEAR.bit.GPIO34=1;             // DESLIGA BKR

// LEDS LÓGICA INVERTIDA POR CONTA DO CIRCUITO DO 724G07
#define DESLIGALED1			GpioDataRegs.GPBSET.bit.GPIO62=1;			// DESLIGALED 1
#define LIGALED1			GpioDataRegs.GPBCLEAR.bit.GPIO62=1;			// LIGALED 1
#define DESLIGALED2			GpioDataRegs.GPASET.bit.GPIO24=1;			// DESLIGALED 2
#define LIGALED2			GpioDataRegs.GPACLEAR.bit.GPIO24=1;			// LIGALED 2
#define DESLIGALED3			GpioDataRegs.GPASET.bit.GPIO28=1;			// DESLIGALED 3
#define LIGALED3			GpioDataRegs.GPACLEAR.bit.GPIO28=1;			// LIGALED 3
#define DESLIGALED4			GpioDataRegs.GPASET.bit.GPIO29=1;			// DESLIGALED 4
#define LIGALED4			GpioDataRegs.GPACLEAR.bit.GPIO29=1;			// LIGALED 4

//SAÍDA ISOLADAS
#define LIGASAIDAISO1	    GpioDataRegs.GPASET.bit.GPIO27=1;			// LIGALED 1
#define DESLIGASAIDAISO1	GpioDataRegs.GPACLEAR.bit.GPIO27=1;			// DESLIGALED 1
#define LIGASAIDAISO2	    GpioDataRegs.GPASET.bit.GPIO31=1;			// LIGALED 2
#define DESLIGASAIDAISO2	GpioDataRegs.GPACLEAR.bit.GPIO31=1;			// DESLIGALED 2
#define LIGASAIDAISO3	    GpioDataRegs.GPBSET.bit.GPIO61=1;			// LIGALED 3
#define DESLIGASAIDAISO3	GpioDataRegs.GPBCLEAR.bit.GPIO61=1;			// DESLIGALED 3
#define LIGASAIDAISO4	    GpioDataRegs.GPBSET.bit.GPIO60=1;			// LIGALED 4
#define DESLIGASAIDAISO4	GpioDataRegs.GPBCLEAR.bit.GPIO60=1;			// DESLIGALED 4

// RELES
#define LIGARELE1	    GpioDataRegs.GPCSET.bit.GPIO85=1;	    		// LIGA RELE1
#define DESLIGARELE1	GpioDataRegs.GPCCLEAR.bit.GPIO85=1;				// DESLIGA RELE1
#define LIGARELE2	    GpioDataRegs.GPASET.bit.GPIO26=1;	    		// LIGA RELE2
#define DESLIGARELE2	GpioDataRegs.GPACLEAR.bit.GPIO26=1;				// DESLIGA RELE2

// BOTOES
#define BOT1	    !GpioDataRegs.GPBDAT.bit.GPIO49			// ESTADO DO BOTAO 1 se apertado BOT1=1
#define BOT2		!GpioDataRegs.GPBDAT.bit.GPIO48			// ESTADO DO BOTAO 2 se apertado BOT2=1


//#############################################################################################

#endif /* DEFINESCONTROLCARDV2_H_ */
