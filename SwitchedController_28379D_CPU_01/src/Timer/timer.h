#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include "F28x_Project.h"
#include <src/settings.h>

namespace Timer
{
    void Configure();

    void SystemEvaluation_Start();

    void ReferenceUpdate_Start();
}

#endif /* SRC_TIMER_H_ */
