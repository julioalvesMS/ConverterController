#ifndef CONFIGURATIONS_H_
#define CONFIGURATIONS_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


//################## ROTINAS CONFIGURAÇÃO DO FUNCIONAMENTO DOS PERIFÉRICOS#############################


//############CONFIGURAÇÃO DOS CANAIS ANALÓGICOS ADC ###################

void Setup_ADC(void);


//###############CONFIGURAÇÃO DOS CANAIS DE PWM####################

void Setup_ePWM(void);

//###############CONFIGURAÇÃO DO DO ENCODER####################

void Setup_EQEP(void);


//#####################CONFIGURA SERIAL##########################
void SCIA_init(void);


//#############################################################################################

#endif /* CONFIGURATIONS_H_ */
