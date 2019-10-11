#include <src/Config/CONFIG_GPIO_V1_LP28377S.h>

void Gpio_setup(void)
{
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA OS PINOS GPIO2-3 e GPIO10-19 COMO PWM						    <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    InitEPwm7Gpio();    // PWM 1
    InitEPwm8Gpio();    // PWM 2

EALLOW;                 // PWM 3
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;    // Disable pull-up on GPIO16 (EPWM9A)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;    // Disable pull-up on GPIO17 (EPWM9B)

    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as EPWM9A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as EPWM9B

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as EPWM9A
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as EPWM9B
EDIS;


    InitEPwm2Gpio();    // PWM 4
    InitEPwm6Gpio();    // PWM 5

EALLOW;                 // PWM 6
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    // Disable pull-up on GPIO18 (EPWM10A)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;    // Disable pull-up on GPIO19 (EPWM10B)

    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as EPWM10A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 1;   // Configure GPIO19 as EPWM10B

    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as EPWM10A
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;   // Configure GPIO19 as EPWM10B
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA LEDS 1 e 2 COMO SAÍDAS							    			<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;
//    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 1;    // LED1/LED19
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1;    // LED2/LED18

//    GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;   // LED1/LED19
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;   // LED2/LED18

//    GpioCtrlRegs.GPDDIR.bit.GPIO123 = 1;    // LED1/LED19
    GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;    // LED2/LED18
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA BOTÕES BOT1 E BOT 2											 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

//EALLOW;
//    GpioCtrlRegs.GPEPUD.bit.GPIO131 = 0;    // BOT1
//    GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;   // BOT1
//    GpioCtrlRegs.GPEDIR.bit.GPIO131 = 0;    // BOT1
//
//    GpioCtrlRegs.GPEPUD.bit.GPIO130 = 0;    // BOT2
//    GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;   // BOT2
//    GpioCtrlRegs.GPEDIR.bit.GPIO130 = 0;    // BOT2
//EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//// CONFIGURA RELE 1 E  RELE 2												 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

//EALLOW;
//    GpioCtrlRegs.GPDPUD.bit.GPIO111 = 1;    //  RELE1
//    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;   //  RELE1
//    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 1;    //  RELE1
//
//    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 1;     //  RELE2
//    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;    //  RELE2
//    GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1;     //  RELE2
//EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA SAIDAS ISOLADAS												 <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><

EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 1;  // SAÍDA ISOLADA 1
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0; // SAÍDA ISOLADA 1
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;  // SAÍDA ISOLADA 1

    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 1;  // SAÍDA ISOLADA 2
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0; // SAÍDA ISOLADA 2
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;  // SAÍDA ISOLADA 2

//    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 1;   // SAÍDA ISOLADA 3
//    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;  // SAÍDA ISOLADA 3
//    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;   // SAÍDA ISOLADA 3

    GpioCtrlRegs.GPCPUD.bit.GPIO69 = 1;   // SAÍDA ISOLADA 4
    GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0;  // SAÍDA ISOLADA 4
    GpioCtrlRegs.GPCDIR.bit.GPIO69 = 1;   // SAÍDA ISOLADA 4
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Configura GPSPI-A on GPIO58 - GPIO61 (DAC)								<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;     // GPIO58  (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;     // GPIO59  (SPISOMIA)
//    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;     // GPIO60  (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;     // GPIO61 (SPISTEA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;   // asynch input
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;   // asynch input
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;   // asynch input
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;   // asynch input

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;   // GPIO58  = SPICLKA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;   // GPIO59  = SPIS0MIA
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;   // GPIO60  = SPISIMIA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3;   // GPIO61 = SPISTEA

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;    // GPIO58  = SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;    // GPIO59  = SPIS0MIA
//    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;    // GPIO60  = SPISIMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;    // GPIO61 = SPISTEA


    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;    // GPIO58  = GPIO ( JUMPER PARA SPICLKB )
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 0;     // GPIO58  = GPIO ( JUMPER PARA SPICLKB )
EDIS;


EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;     // GPIO63  (SPISIMOB)
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;     // GPIO64  (SPISOMIB)
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;     // GPIO65  (SPICLKB)
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;     // GPIO66  (SPISTEB)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;   // asynch input
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;   // asynch input
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;   // asynch input
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3;   // asynch input

    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;   // GPIO63  = SPICLKB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;   // GPIO64  = SPIS0MIB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;   // GPIO65  = SPISIMIB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 3;   // GPIO66  = SPISTEB

    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;    // GPIO63  = SPICLKB
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;    // GPIO64  = SPIS0MIB
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;    // GPIO65  = SPISIMIB
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;    // GPIO66  = SPISTEB

    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0;    // GPIO91  = GPIO ( JUMPER PARA SPISTEB )
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = 0;     // GPIO91  = GPIO ( JUMPER PARA SPISTEB )
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA BREAKS E RESETES DOS VSCS										<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;   //  BRK
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;  //  BRK
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;   //  BRK

    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 1;   //  RESETVSC
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0;  //  RESETVSC
    GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1;   //  RESETVSC

//    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;   //  BRK2
//    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;  //  BRK2
//    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;   //  BRK2

    GpioCtrlRegs.GPCPUD.bit.GPIO73 = 1;  //  RESETEVSC2
    GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0; //  RESETEVSC2
    GpioCtrlRegs.GPCDIR.bit.GPIO73 = 1;  //  RESETEVSC2
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// CONFIGURA ENTRADAS DE TRIPZONE E RESET DO TZ ENTRADAS ANALÓGICAS		   <
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;     //PULL UP
//    GpioCtrlRegs.GPDPUD.bit.GPIO97 = 0;     // TZ1  SENSORES ESQUERDA
//    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;     // TZ2  ERROVSC2
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;     // TZ3  ERRO VSC
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;     // TZ4  SENSORES DIREITA
EDIS;

EALLOW;     // Input Qualification 00=sync 01=3 ciclos 02=6 ciclo   03= no sync
//    GpioCtrlRegs.GPDQSEL1.bit.GPIO97 = 2;   // TZ1  SENSORES ESQUERDA
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 2;   // TZ2  ERROVSC2
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // TZ3  ERROVSC
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 2;   // TZ4  SENSORES DIREITA


//    GpioCtrlRegs.GPDMUX1.bit.GPIO97 = 0;    // GPIO12 = TZ1 SENSORES ESQUERDA
//    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;    // GPIO13 = TZ2 ERROVSC2
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // GPIO14 = TZ3 ERROVSC
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;    // GPIO15 = TZ4 SENSORES DIREITA
EDIS;

EALLOW;     //Configura GPIO72 RESETTZ
     GpioCtrlRegs.GPCPUD.bit.GPIO72 = 1;   //  RESETTZ_SENSORES
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 0;  //  RESETTZ_SENSORES
     GpioCtrlRegs.GPCDIR.bit.GPIO72 = 1;   //  RESETTZ_SENSORES
 EDIS;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  CONFIGURA CANAIS DE RS232												<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// ( SCI-B on GPIO86 - GPIO87)
EALLOW;
    // Enable SCI-B on GPIO86 - GPIO87
    GpioCtrlRegs.GPCPUD.bit.GPIO87 = 0;     // Enable pullup on GPIO87
    GpioCtrlRegs.GPCGMUX2.bit.GPIO87 = 1;   // GPIO87 = SCIRXDB_DSP
    GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 1;    // GPIO87 = SCIRXDB_DSP
    GpioCtrlRegs.GPCPUD.bit.GPIO86 = 0;     // Enable pullup on GPIO86
    GpioCtrlRegs.GPCGMUX2.bit.GPIO86 = 1;   // GPIO86 = SCITXDB_DSP
    GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 1;    // GPIO86 = SCITXDB_DSP
EDIS;

// ( SCI-C on GPIO89 - GPIO90)
EALLOW;
    // Enable SCI-C on GPIO89 - GPIO90
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = 0;     // Enable pullup on GPIO90
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = 1;   // GPIO90 = SCIRXDB_DSP
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = 2;    // GPIO90 = SCIRXDB_DSP
    GpioCtrlRegs.GPCPUD.bit.GPIO89 = 0;     // Enable pullup on GPIO89
    GpioCtrlRegs.GPCGMUX2.bit.GPIO89 = 1;   // GPIO89 = SCITXDB_DSP
    GpioCtrlRegs.GPCMUX2.bit.GPIO89 = 2;    // GPIO89 = SCITXDB_DSP
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  CONFIGURA CANAIS DO ENCODER INCREMENTAR EQEP1	on GPIO10 - GPIO13
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EQEP1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;    // Disable pull-up on GPIO12 (EQEP1S)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;    // Disable pull-up on GPIO13 (EQEP1I)

    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 0;   // Sync GPIO10 to SYSCLK  (EQEP1A)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0;   // Sync GPIO11 to SYSCLK  (EQEP1B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 0;   // Sync GPIO12 to SYSCLK  (EQEP1S)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 0;   // Sync GPIO13 to SYSCLK  (EQEP1I)

    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EQEP1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;    // Configure GPIO10 as EQEP1A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EQEP1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;    // Configure GPIO11 as EQEP1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EQEP1S
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;    // Configure GPIO12 as EQEP1S
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 1;   // Configure GPIO13 as EQEP1I
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;    // Configure GPIO13 as EQEP1I
EDIS;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  GPIO NÃO UTILIZADOS	CONFIGURADOS COMO SAÍDA PULL UP						<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

EALLOW;     // PULL UP
    GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;  // Livre
    GpioCtrlRegs.GPCPUD.bit.GPIO78 = 0;  // Livre
    GpioCtrlRegs.GPDPUD.bit.GPIO99 = 0;  // Livre
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO38 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO44 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO45 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO68 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO74 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO75 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO79 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO88 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO95 = 0;
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPDPUD.bit.GPIO96 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO98 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO100 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO101 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO102 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO103 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO106 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO107 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO108 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO109 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO110 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO111 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO112 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO113 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO114 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO115 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO116 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO117 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO118 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO119 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO120 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO121 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO122 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO125 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO126 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO127 = 0;
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPEPUD.bit.GPIO128 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO129 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO131 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO132 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO133 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO134 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO135 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO136 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO137 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO138 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO139 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO140 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO141 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO142 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO143 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO144 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO145 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO146 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO147 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO148 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO149 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO150 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO151 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO152 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO153 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO154 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO155 = 0;
    GpioCtrlRegs.GPEPUD.bit.GPIO156 = 0;
EDIS;

EALLOW;     // PULL UP
    GpioCtrlRegs.GPFPUD.bit.GPIO161 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO162 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO163 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO164 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO165 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO166 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO167 = 0;
    GpioCtrlRegs.GPFPUD.bit.GPIO168 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0;  // Livre
    GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0;  // Livre
    GpioCtrlRegs.GPDMUX1.bit.GPIO99 = 0;  // Livre
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO88 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO95 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPDMUX1.bit.GPIO96 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO98 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO100 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO101 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO102 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO103 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO106 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO107 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO108 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO109 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO110 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO112 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO113 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO114 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO115 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO116 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO117 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO118 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO119 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO120 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO121 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO125 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO127 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPEMUX1.bit.GPIO128 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO129 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO132 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO133 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO134 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO135 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO136 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO137 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO138 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO139 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO140 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO141 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO142 = 0;
    GpioCtrlRegs.GPEMUX1.bit.GPIO143 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO144 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO145 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO146 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO147 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO148 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO149 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO150 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO151 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO152 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO153 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO154 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO155 = 0;
    GpioCtrlRegs.GPEMUX2.bit.GPIO156 = 0;
EDIS;

EALLOW;     // DEFINE COM GPIO
    GpioCtrlRegs.GPFMUX1.bit.GPIO161 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO162 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO163 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO164 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO165 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO166 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO167 = 0;
    GpioCtrlRegs.GPFMUX1.bit.GPIO168 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPCDIR.bit.GPIO71 = 0;  // Livre
    GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0;  // Livre
    GpioCtrlRegs.GPDDIR.bit.GPIO99 = 0;  // Livre
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO37 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO38 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO45 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO46 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO47 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO68 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO70 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO88 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO92 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO95 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPDDIR.bit.GPIO96 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO98 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO100 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO101 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO102 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO103 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO106 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO107 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO108 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO109 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO110 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO112 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO113 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO114 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO115 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO116 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO117 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO118 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO119 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO120 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO121 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO122 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO123 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO125 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO127 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPEDIR.bit.GPIO128 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO129 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO131 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO132 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO133 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO134 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO135 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO136 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO137 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO138 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO139 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO140 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO141 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO142 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO143 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO144 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO145 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO146 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO147 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO148 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO149 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO150 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO151 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO152 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO153 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO154 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO155 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO156 = 0;
EDIS;

EALLOW;     // DEFINE COM SAÍDA
    GpioCtrlRegs.GPFDIR.bit.GPIO161 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO162 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO163 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO164 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO165 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO166 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO167 = 0;
    GpioCtrlRegs.GPFDIR.bit.GPIO168 = 0;
EDIS;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

}

