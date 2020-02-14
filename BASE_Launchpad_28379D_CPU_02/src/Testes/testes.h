#ifndef SRC_TESTES_TESTES_H_
#define SRC_TESTES_TESTES_H_

#include <string.h>
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Config/SPI_DAC.h>
#include <src/Config/COM_SERIAL.h>


extern int grupo_dac;

extern float Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern float Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;

extern float pos, dir;
extern float vel_rpm_inst;
extern float vel_rpm;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_DAC_SPI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa a DAC por SPI                                                              %
// SÃO CRIADAS SAIDAS EM RAMPAS QUE SÃO ENVIADAS PARA OS CANAIS DA DAC                         %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_DAC_SPI(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA CANAIS DE ENTRADAS ANALÓGICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função ativa a interrupção do AD adc_isr onde são lidos o 16 canais analógicos           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_ENTRADAS_ANALOGICAS(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA ENCODER EQEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função ativa a interrupção do AD adc_isr onde é medida a velocidade atraves da função    %
// medir_velocidade(angulo, direção)                                                            %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_ENCODER(void);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA RS232 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta função testa a comunicação por RS232    SCI-A on GPIO35 - GPIO36                         %
// Envia a mensagem TX-OK 20 vezes e espera a chegada do caracter  T definido na funçã SCI_RX   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_RS232(void);


#endif /* SRC_TESTES_TESTES_H_ */
