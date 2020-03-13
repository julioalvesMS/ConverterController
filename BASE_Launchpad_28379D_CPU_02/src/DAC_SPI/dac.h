#ifndef SRC_DAC_SPI_H_
#define SRC_DAC_SPI_H_

#include "F28x_Project.h"
#include <src/Config/SPI_DAC.h>

#define DAC_CHANNEL_COUNT 2


extern double Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern double Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;


void DAC_SPI_Configure(void);

void DAC_SPI_SendData(Uint16 opt);


#endif /* SRC_DAC_SPI_H_ */
