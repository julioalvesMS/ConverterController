#ifndef SRC_OPERATION_MANAGEMENT_MANAGER_H_
#define SRC_OPERATION_MANAGEMENT_MANAGER_H_

#include <src/Controller/controller.h>
#include <src/Controller/ClassicController/voltage_controller.h>
#include <src/Controller/ClassicController/voltage_current_controller.h>
#include <src/Converter/base_converter.h>
#include <src/Equilibrium/equilibrium.h>
#include <src/Equilibrium/current_correction.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Equilibrium/partial_information.h>
#include <src/IPC/protocol.h>
#include <src/Protection/protection.h>
#include <src/Relay/relay.h>
#include <src/Sensor/sensor.h>
#include <src/Switch/switch.h>

using namespace BaseConverter;
using namespace Controller;

namespace Manager
{
    enum OperationState
    {
        OS_OFF = 0,
        OS_RUNNING = 1,
        OS_STARTING_PRE_LOAD = 2,
        OS_PRE_LOAD = 3,
        OS_ENDING_PRE_LOAD = 4,
        OS_CHANGING_CONVERTER_CONTROLLER = 5
    };


    void ExecuteCommand(void);


    void ChangeConverter(ConverterID newConverter);


    void ChangeController(ControlStrategy newController);


    void CompleteConverterControllerChange(void);


    void EnableOperation(void);


    void DisableOperation(void);


    void ContinuePreLoad(void);


    OperationState* GetCurrentState(void);

}

#endif /* SRC_OPERATION_MANAGEMENT_MANAGER_H_ */
