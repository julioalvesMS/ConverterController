#ifndef CONFIGURATIONS_H_
#define CONFIGURATIONS_H_

#include "F28x_Project.h"   // Device Headerfile and Examples Include File
#include <src/Config/DEFINES_LP28377S.h>


//################## ROTINAS CONFIGURA��O DO FUNCIONAMENTO DOS PERIF�RICOS #############################


//############CONFIGURA��O DOS CANAIS ANAL�GICOS ADC ###################

void Setup_ADC(void);


//###############CONFIGURA��O DOS CANAIS DE PWM####################

void Setup_ePWM(void);

//###############CONFIGURA��O DO DO ENCODER####################

void Setup_EQEP(void);


//#####################CONFIGURA SERIAL##########################
void SCIB_init(void);

void SCIC_init(void);

//#############################################################################################

#endif /* CONFIGURATIONS_H_ */
