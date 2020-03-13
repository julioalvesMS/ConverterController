#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include "F28x_Project.h"
#include <src/Config/PWM_DAC.h>

extern double Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern double Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;

void DAC_PWM_Configure(void);

void DAC_PWM_SendData(Uint16 opt);


#endif /* SRC_DAC_H_ */
