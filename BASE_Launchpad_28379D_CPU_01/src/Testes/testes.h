#ifndef SRC_TESTES_TESTES_H_
#define SRC_TESTES_TESTES_H_

#include <string.h>
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include "../Config/CONFIG_GPIO_DUAL_LP28379D.h"


extern int grupo_dac;

extern double Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern double Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;

extern double pos, dir;
extern double vel_rpm_inst;
extern double vel_rpm;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_LEDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa os pinos de GPIO LEDS 1-4 19-21  DA PLACA ligados aos GPIO 67,69,71,71     %
//OS LEDS PISCA 20 VEZES A CADA CHAMADA  DESTA FUNÇÃO                                          %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_LEDS(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_BOTOESS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa os pinos de GPIO 49 e 48, que estão ligados aos botões 1 e 2               %
//Os LEDS 1,2 -19,18 serão ligados enquanto o botão BOT1 for pressionado                       %
//Os LEDS 3,4- 20,21 serão ligados enquanto o botão BOT2 for pressionado                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BOTOES(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_RELÉS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa os pinos de GPIO 23 e 16, que estão ligados aos relés RELE1 E RELE2        %
//Os relés são ligados e desligados a cada 1 segundo                                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_RELES(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_PWN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa os canais os 12 canais ePWM                                                %
// OS PWM 1-6 são acionados com indice de modulação (m) que varia de de-1 até1                 %
// Caso o botão B1 seja acionado o Trip zone dos sensores é acionado                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_PWM(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS_RESET/BKR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa as saídas de reset e de break dos VSCs                                     %
// Caso o botão B1 seja acionado as saidas RESETVSC(gpio62) E RESETVSC2 são acionadas          %
// Caso o botão B2 seja acionado as saidas BRK e BKR2 são acionadas                            %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BKR_RST(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS ISOLADAS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa os pinos de GPIO 27,21,61,60 que controlam as SAIDAS ISOLADAS1-4           %
//AS SAIDAS PISCAM 20 VEZES A CADA CHAMADA  DESTA FUNÇÃO                                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_SAIDAS_ISOLADAS(void);


#endif /* SRC_TESTES_TESTES_H_ */
