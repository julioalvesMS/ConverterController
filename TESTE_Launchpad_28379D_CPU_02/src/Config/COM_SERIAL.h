#ifndef COM_SERIAL_H_
#define COM_SERIAL_H_

#include "F28x_Project.h"

extern char message[8];
extern Uint16 buffer;

//
// TRANSMITE PELA SERIAL - RS232 B (DIREITA)
//
void SCIB_TX(void);

//
// RECEBE PELA SERIAL - RS232 B (DIREITA)
//
void SCIB_RX(void);

//
// TRANSMITE PELA SERIAL - RS232 C (ESQUERDA)
//
void SCIC_TX(void);

//
// RECEBE PELA SERIAL - RS232 C (ESQUERDA)
//
void SCIC_RX(void);

#endif /* COM_SERIAL_H_ */
