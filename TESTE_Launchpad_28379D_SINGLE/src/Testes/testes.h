#ifndef SRC_TESTES_TESTES_H_
#define SRC_TESTES_TESTES_H_

#include <string.h>
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Config/CONFIG_GPIO_V1_LP28379D.h>
#include <src/Config/SPI_DAC.h>
#include <src/Config/COM_SERIAL.h>


extern int grupo_dac;

extern float Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern float Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;

extern float pos, dir;
extern float vel_rpm_inst;
extern float vel_rpm;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_LEDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO LEDS 1-4 19-21  DA PLACA ligados aos GPIO 67,69,71,71     %
//OS LEDS PISCA 20 VEZES A CADA CHAMADA  DESTA FUN��O                                          %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_LEDS(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_BOTOESS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 49 e 48, que est�o ligados aos bot�es 1 e 2               %
//Os LEDS 1,2 -19,18 ser�o ligados enquanto o bot�o BOT1 for pressionado                       %
//Os LEDS 3,4- 20,21 ser�o ligados enquanto o bot�o BOT2 for pressionado                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BOTOES(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_REL�S%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 23 e 16, que est�o ligados aos rel�s RELE1 E RELE2        %
//Os rel�s s�o ligados e desligados a cada 1 segundo                                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_RELES(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_DAC_SPI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a DAC por SPI                                                              %
// S�O CRIADAS SAIDAS EM RAMPAS QUE S�O ENVIADAS PARA OS CANAIS DA DAC                         %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_DAC_SPI(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_PWN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os canais os 12 canais ePWM                                                %
// OS PWM 1-6 s�o acionados com indice de modula��o (m) que varia de de-1 at�1                 %
// Caso o bot�o B1 seja acionado o Trip zone dos sensores � acionado                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_PWM(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS_RESET/BKR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa as sa�das de reset e de break dos VSCs                                     %
// Caso o bot�o B1 seja acionado as saidas RESETVSC(gpio62) E RESETVSC2 s�o acionadas          %
// Caso o bot�o B2 seja acionado as saidas BRK e BKR2 s�o acionadas                            %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BKR_RST(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS ISOLADAS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 27,21,61,60 que controlam as SAIDAS ISOLADAS1-4           %
//AS SAIDAS PISCAM 20 VEZES A CADA CHAMADA  DESTA FUN��O                                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_SAIDAS_ISOLADAS(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA CANAIS DE ENTRADAS ANAL�GICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde s�o lidos o 16 canais anal�gicos           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_ENTRADAS_ANALOGICAS(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA ENCODER EQEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde � medida a velocidade atraves da fun��o    %
// medir_velocidade(angulo, dire��o)                                                            %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_ENCODER(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA RS232 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a comunica��o por RS232    SCI-A on GPIO35 - GPIO36                         %
// Envia a mensagem TX-OK 20 vezes e espera a chegada do caracter  T definido na fun�� SCI_RX   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_RS232(void);


#endif /* SRC_TESTES_TESTES_H_ */
