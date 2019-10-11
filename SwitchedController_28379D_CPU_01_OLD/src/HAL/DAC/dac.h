#ifndef SRC_HAL_DAC_H_
#define SRC_HAL_DAC_H_

#include "F28x_Project.h"
#include <src/settings_cpu_01.h>

//
// Defines
//
#define REFERENCE_VDAC      0
#define REFERENCE_VREF      1
#define DACA                1
#define DACB                2
#define DACC                3
#define REFERENCE           REFERENCE_VREF
#define DAC_NUM_IL          DACA
#define DAC_NUM_VOUT        DACB

namespace DAC_HAL
{
    void Configure(void);

    void Update(unsigned int IL, unsigned int VOUT);
}

#endif /* SRC_HAL_DAC_H_ */
