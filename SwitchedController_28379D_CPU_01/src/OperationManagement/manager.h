#ifndef SRC_OPERATION_MANAGEMENT_MANAGER_H_
#define SRC_OPERATION_MANAGEMENT_MANAGER_H_

#include <src/Converter/base_converter.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Relay/relay.h>

using namespace BaseConverter;

namespace Manager
{
    enum OperationState
    {
        OS_OFF = 0,
        OS_RUNNING = 1,
        OS_STARTING_PRE_LOAD = 2,
        OS_PRE_LOAD = 3,
        OS_ENDING_PRE_LOAD = 4,
        OS_CHANGING_CONVERTER = 5
    };


    void ChangeConverter(ConverterID newConverter);


    void CompleteConverterChange(void);


    void EnableOperation();


    void DisableOperation();


    void ContinuePreLoad(void);


    OperationState* GetCurrentState(void);

}

#endif /* SRC_OPERATION_MANAGEMENT_MANAGER_H_ */
