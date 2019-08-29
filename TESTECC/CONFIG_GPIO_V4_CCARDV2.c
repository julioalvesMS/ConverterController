//###########################################################################
//
// Cabeçalho:   Config_Gpio.DFIG.c
//
// Título:  Config_Gpio.DFIG.c

//Descrição : Este cabeçalho contém as funções necessárias para configurar o
//pinos do DSP para  trabalhar com a placa de interface desenvolvida e a placa CONTROLCARV2.
//
//
///###########################################################################
// Author: TÁRCIO ANDRÉ 12/12/2017
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

void Gpio_setup1(void)
{

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA OS PINOS GPIO0-11 COMO PWM										<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


 EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B=VSC
   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;  // GPIO6 = PWM4A=VSC2
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;  // GPIO7 = PWM4B=VSC2
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;  // GPIO8 = PWM5A=VSC2
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;  // GPIO9 = PWM5B=VSC2
   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; // GPI010 = PWM6A=VSC2
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // GPIO11 = PWM6B=VSC2
   EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA LEDS 1,2,3,4 COMO SAÍDAS										<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

 EALLOW;
 	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1;   // LED1/LED19
 	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;   // LED2/LED18
 	//GpioCtrlRegs.GPAPUD.bit.GPIO28 = 1;   // LED3/LED21
 	//GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;   // LED4/LED20

 	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;  // LED1/LED19
 	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // LED2/LED18
 	//GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  // LED3/LED21
 	//GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  // LED4/LED20

 	GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;   // LED1/LED19
 	GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;   // LED2/LED18
 	//GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;   // LED3/LED21
 	//GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;   // LED4/LED20
 EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA BOTÕES BOT1 E BOT 2											 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

 EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;  // BOT1
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 =0;  // BOT1
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;  // BOT1

    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;  // BOT2
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 =0;  // BOT2
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;  // BOT2

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//// CONFIGURA RELE 1 E  RELE 2												 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

     EALLOW;

      GpioCtrlRegs.GPCPUD.bit.GPIO85 = 1;   //  RELE1
      GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;  //  RELE1
      GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;   //  RELE1

      GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1;   //  RELE2
      GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;  //  RELE2
      GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;   //  RELE2
      EDIS;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA SAIDAS ISOLADAS												 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

      EALLOW;
      GpioCtrlRegs.GPAPUD.bit.GPIO27 = 1;   // SAÍDA ISOLADA 1
      GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;  // SAÍDA ISOLADA 1
      GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;   // SAÍDA ISOLADA 1
      GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;   // SAÍDA ISOLADA 2
      GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // SAÍDA ISOLADA 2
      GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;   // SAÍDA ISOLADA 2

      GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;   // SAÍDA ISOLADA 4
      GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;  // SAÍDA ISOLADA 4
      GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;   // SAÍDA ISOLADA 4

      GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;   // SAÍDA ISOLADA 3
      GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;  // SAÍDA ISOLADA 3
      GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;   // SAÍDA ISOLADA 3

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Configura GPSPI-A on GPIO54 - GPIO57(DAC)								<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      EALLOW;
      GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // GPIO16 (SPISIMOA)
      GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // GPIO17 (SPISOMIA)
      GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // GPIO18 (SPICLKA)
      GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // GPIO19 (SPISTEA)
      GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // asynch input
      GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // asynch input
      GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // asynch input
      GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // asynch input
      GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // GPIO16 = SPICLKA
      GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // GPIO17 = SPIS0MIA
      GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // GPIO18 = SPISIMIA
      GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;  // GPIO19 = SPISTEA
      EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA BREAKS E RESETES DOS VSCS										<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
EALLOW;

      GpioCtrlRegs.GPBPUD.bit.GPIO59 = 1;   //  BRK
      GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;  //  BRK
      GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;   //  BRK

      GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;   //  RESETVSC
      GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;  //  RESETVSC
      GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;   //  RESETVSC

      GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;   //  BRK2
      GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;  //  BRK2
      GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;   //  BRK2

      GpioCtrlRegs.GPCPUD.bit.GPIO86 = 1;   //  RESETEVSC2
      GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;  //  RESETEVSC2
      GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;   //  RESETEVSC2

EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA ENTRADAS DE TRIPZONE E RESET DO TZ ENTRADAS ANALÓGICAS		   <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;				//PULL UP
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // TZ1  SENSORES ESQUERDA
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // TZ2  ERROVSC2
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // TZ3  ERRO VSC
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // TZ4  SENSORES DIREITA

  EDIS;

  EALLOW;
    		// Input Qualification 00=sync 01=3 ciclos 02=6 ciclo   03= no sync
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 2; // TZ1  SENSORES ESQUERDA
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 2; // TZ2  ERROVSC2
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 2; // TZ3  ERROVSC
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 2; // TZ4  SENSORES DIREITA


    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO12 = TZ1 SENSORES ESQUERDA
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // GPIO13 = TZ2 ERROVSC2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;  // GPIO14 = TZ3 ERROVSC
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // GPIO15 = TZ4 SENSORES DIREITA

  EDIS;

EALLOW;
  	  	  	  	  //Configura GPIO34 RESETTZ
     GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;   //  RESETTZ_SENSORES
     GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  //  RESETTZ_SENSORES
     GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   //  RESETTZ_SENSORES
 EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  CONFIGURA CANAIS DE RS232												<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// ( SCI-A on GPIO35 - GPIO36)

EALLOW;
   // Enable SCI-A on GPIO35 - GPIO36
   GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Enable pullup on GPIO19
   //GpioCtrlRegs.GPAQSEL1.bit.GPIO29 = 3; // Asynch input
   GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // GPIO36 = SCIRXDA_DSP
   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Enable pullup on GPIO18
   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // GPIO35 = SCITXDA_DSP
EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  CONFIGURA CANAIS DO ENCODER INCREMENTAR EQEP1	on GPIO50 - GPIO53   	<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;

      GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;  //
      GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;  // GPIO20 = EQEP1A

      GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;  //
      GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;  // GPIO21 = EQEP1B

      GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;  //
      GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;  // GPIO22 = EQEP1S

      GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;  //
      GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;  // GPIO23 = EQEP1L

 EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  GPIO NÃO UTILIZADOS I2C													<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Sets the GPIO pins Enable I2C-A(DAC) on GPIO32 - GPIO33
 EALLOW;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCL
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3; // Asynch
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPI32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPI32=SDA

EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  GPIO NÃO UTILIZADOS	CONFIGURADOS COMO SAÍDA PULL UP						<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/////////////////////////////////////////////////PULL UP
EALLOW;

   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;//Livre
   GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;//Livre
   GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;//Livre
EDIS;
EALLOW;
   GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO38 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO44 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO45 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;//Livre
EDIS;
EALLOW;
   GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO67 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO68 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO72 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO73 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO74 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO75 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO78 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO79 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0;
   GpioCtrlRegs.GPCPUD.bit.GPIO84 = 0;//Livre* tem só na placa do card
   GpioCtrlRegs.GPCPUD.bit.GPIO87 = 0;
EDIS;

////////////////////////////////////////////DEFINE COM GPIO
EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;//Livre
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;//Livre
   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;//Livre

EDIS;
EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;//Livre
EDIS;
EALLOW;

   GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0;
   GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;
EDIS;
EALLOW;
   GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;//Livre
   GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;
EDIS;

////////////////////////////////////////////DEFINE COM SAÍDA
EALLOW;
   GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;//Livre
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;//Livre
   GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;//Livre
EDIS;
EALLOW;
   GpioCtrlRegs.GPBDIR.bit.GPIO37 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO38 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO44 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO45 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO46 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO47 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO57 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0;//Livre
EDIS;
EALLOW;
   GpioCtrlRegs.GPCDIR.bit.GPIO64 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO65 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO66 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO66 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO68 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO69 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO70 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO71 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO72 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO73 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0;
EDIS;
EALLOW;
   GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO84 = 0;//Livre
   GpioCtrlRegs.GPCDIR.bit.GPIO87 = 0;

   EDIS;
   //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

}

