#ifndef DEFINES_LP28379D_H_
#define DEFINES_LP28379D_H_

#include "F28x_Project.h"

//####################################################### DEFINIÇÕES PARA OS PWMs ###############################

#define EPWM1_Regs       EPwm7Regs
#define EPWM2_Regs       EPwm8Regs
#define EPWM3_Regs       EPwm9Regs
#define EPWM4_Regs       EPwm2Regs
#define EPWM5_Regs       EPwm6Regs
#define EPWM6_Regs       EPwm10Regs

//####################################################### DEFINIÇÕES PARA AS GPIOs ###############################

// LEDs
#define GPIO_LED1   GPIO62
//#define GPIO_LED2   GPIO122

// Botões
//#define GPIO_BOT1   GPIO131
//#define GPIO_BOT2   GPIO130

// Reset VSC 1 e 2
#define GPIO_RVS1   GPIO92
#define GPIO_RVS2   GPIO73

// Erro VSC 1 e 2
#define GPIO_EVS1   GPIO20
//#define GPIO_EVS2   GPIO14

// Brakes 1 e 2
#define GPIO_BRK1   GPIO21
//#define GPIO_BRK2   GPIO15

// Reset Tripzone
#define GPIO_RTZ    GPIO72

// Saídas isoladas
#define GPIO_ISO1   GPIO42
#define GPIO_ISO2   GPIO43
//#define GPIO_ISO3   GPIO66
#define GPIO_ISO4   GPIO69

// Reles 1 e 2
//#define GPIO_REL1   GPIO111
//#define GPIO_REL2   GPIO94


//####################################################### DEFINIÇÕES DE NOMES PARA AÇOES ###############################

// RESET E BKR DOS VSC
#define LIGARESETVSC	    GpioDataRegs.GPCSET.bit.GPIO_RVS1   = 1     // RESETA VSC
#define DESLIGARESETVSC	    GpioDataRegs.GPCCLEAR.bit.GPIO_RVS1 = 1     // DESLIGA VSC

#define LIGARESETVSC2	    GpioDataRegs.GPCSET.bit.GPIO_RVS2   = 1     // RESETA VSC2
#define DESLIGARESETVSC2	GpioDataRegs.GPCCLEAR.bit.GPIO_RVS2 = 1     // DESLIGA VSC2

#define LIGABKR       	    GpioDataRegs.GPASET.bit.GPIO_BRK1   = 1     // RESETA BKR
#define DESLIGABKR       	GpioDataRegs.GPACLEAR.bit.GPIO_BRK1 = 1     // DESLIGA BKR

//#define LIGABKR2	        GpioDataRegs.GPASET.bit.GPIO_BRK2   = 1     // RESETA BKR2
//#define DESLIGABKR2	    	GpioDataRegs.GPACLEAR.bit.GPIO_BRK2 = 1     // DESLIGA BKR2

// RESET DO TZ
#define DESLISGATZSENSORES  GpioDataRegs.GPCSET.bit.GPIO_RTZ    = 1     // RESETA BKR
#define LISGATZSENSORES     GpioDataRegs.GPCCLEAR.bit.GPIO_RTZ  = 1     // DESLIGA BKR

// LEDS LÓGICA INVERTIDA POR CONTA DO CIRCUI0O DO 724G07
#define DESLIGALED1			GpioDataRegs.GPBSET.bit.GPIO_LED1   = 1     // DESLIGALED 1
#define LIGALED1			GpioDataRegs.GPBCLEAR.bit.GPIO_LED1 = 1     // LIGALED 1
//#define DESLIGALED2			GpioDataRegs.GPDSET.bit.GPIO_LED2   = 1     // DESLIGALED 2
//#define LIGALED2			GpioDataRegs.GPDCLEAR.bit.GPIO_LED2 = 1     // LIGALED 2

//SAÍDA ISOLADAS
#define LIGASAIDAISO1	    GpioDataRegs.GPBSET.bit.GPIO_ISO1   = 1     // LIGALED 1
#define DESLIGASAIDAISO1	GpioDataRegs.GPBCLEAR.bit.GPIO_ISO1 = 1     // DESLIGALED 1
#define LIGASAIDAISO2	    GpioDataRegs.GPBSET.bit.GPIO_ISO2   = 1     // LIGALED 2
#define DESLIGASAIDAISO2	GpioDataRegs.GPBCLEAR.bit.GPIO_ISO2 = 1     // DESLIGALED 2
//#define LIGASAIDAISO3	    GpioDataRegs.GPCSET.bit.GPIO_ISO3   = 1     // LIGALED 3
//#define DESLIGASAIDAISO3	GpioDataRegs.GPCCLEAR.bit.GPIO_ISO3 = 1     // DESLIGALED 3
#define LIGASAIDAISO4	    GpioDataRegs.GPCSET.bit.GPIO_ISO4   = 1     // LIGALED 4
#define DESLIGASAIDAISO4	GpioDataRegs.GPCCLEAR.bit.GPIO_ISO4 = 1     // DESLIGALED 4

// RELES
//#define LIGARELE1	    GpioDataRegs.GPDSET.bit.GPIO_REL1   = 1         // LIGA RELE1
//#define DESLIGARELE1	GpioDataRegs.GPDCLEAR.bit.GPIO_REL1 = 1         // DESLIGA RELE1
//#define LIGARELE2	    GpioDataRegs.GPCSET.bit.GPIO_REL2   = 1         // LIGA RELE2
//#define DESLIGARELE2	GpioDataRegs.GPCCLEAR.bit.GPIO_REL2 = 1         // DESLIGA RELE2

// BOTOES
//#define BOT1	    !GpioDataRegs.GPEDAT.bit.GPIO_BOT1                  // ESTADO DO BOTAO 1 se apertado BOT1=1
//#define BOT2		!GpioDataRegs.GPEDAT.bit.GPIO_BOT2                  // ESTADO DO BOTAO 2 se apertado BOT2=1


//#############################################################################################

#endif /* DEFINES_LP28379D_H_ */
