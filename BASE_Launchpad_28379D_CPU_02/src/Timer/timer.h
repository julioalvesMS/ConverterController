#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include "F28x_Project.h"


#define SYSTEM_EVALUATION_PERIOD 100000     // Period in Microseconds


void Timer_Configure(void);


void Timer_CommunicationTimer_Start(void);


#endif /* SRC_TIMER_H_ */
