#include <src/OperationManagement/manager.h>

extern double *Vin, *Vout;
extern bool ConverterEnabled;
extern bool OutputLoadStep;
extern bool ModeHoppingEnabled;
extern ConverterID activeConverter;
extern ControlStrategy controlStrategy;
extern Equilibrium::EquilibriumMethod CorrectionMethod;
extern Protection::Problem protection;

extern Protocol::CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;

static int lastBufferIndex = 0;

namespace Manager
{
    OperationState CurrentState = OS_OFF;


    void ExecuteCommand(void)
    {
        if (lastBufferIndex==IPC_BufferIndex)
            return;

        switch(IPC_CommandBuffer[lastBufferIndex])
        {
        case Protocol::EnableOperation:
            Manager::EnableOperation();
            break;
        case Protocol::DisableOperation:
            Manager::DisableOperation();
            break;
        case Protocol::IncreaseDacChannel:
            break;
        case Protocol::DecreaseDacChannel:
            break;
        case Protocol::RampIncreaseReference:
            Vref += 0.002;
            if(Vref > PROTECTION_VOUT_MAX) Vref = PROTECTION_VOUT_MAX;
            break;
        case Protocol::RampDecreaseReference:
            Vref -= 0.002;
            if(Vref < 0) Vref = 0;
            break;
        case Protocol::StepIncreaseReference:
            Vref += 5;
            if(Vref > PROTECTION_VOUT_MAX) Vref = PROTECTION_VOUT_MAX;
            break;
        case Protocol::StepDecreaseReference:
            Vref -= 5;
            if(Vref < 0) Vref = 0;
            break;
        case Protocol::ResetReference:
            Vref = 0;
            break;
        case Protocol::EmergencyButtonProtection:
            protection = Protection::FAULT_EMERGENCY_STOP;
            break;
        case Protocol::ResetProtection:
            protection = Protection::NONE;
            break;

        case Protocol::ConverterBuck:
            Manager::ChangeConverter(ID_Buck);
            break;
        case Protocol::ConverterBoost:
            Manager::ChangeConverter(ID_Boost);
            break;
        case Protocol::ConverterBuckBoost:
            Manager::ChangeConverter(ID_BuckBoost);
            break;
        case Protocol::ConverterBuckBoost3:
            Manager::ChangeConverter(ID_BuckBoost3);
            break;

        case Protocol::ControllerClassic:
            Manager::ChangeController(CS_CLASSIC_PWM);
            break;
        case Protocol::ControllerContinuous1:
            Manager::ChangeController(CS_CONTINUOUS_THEOREM_1);
            break;
        case Protocol::ControllerContinuous2:
            Manager::ChangeController(CS_CONTINUOUS_THEOREM_2);
            break;
        case Protocol::ControllerDiscrete1:
            Manager::ChangeController(CS_DISCRETE_THEOREM_1);
            break;
        case Protocol::ControllerClassicVC:
            Manager::ChangeController(CS_CLASSIC_VC_PWM);
            break;
        case Protocol::ControllerLimitCycleCost:
            Manager::ChangeController(CS_LIMIT_CYCLE_COST);
            break;
        case Protocol::ControllerLimitCycleH2:
            Manager::ChangeController(CS_LIMIT_CYCLE_H2);
            break;
        case Protocol::ControllerLimitCycleHinf:
            Manager::ChangeController(CS_LIMIT_CYCLE_Hinf);
            break;
        case Protocol::ControllerStateFeedbackH2:
            Manager::ChangeController(CS_STATE_H2_PWM);
            break;
        case Protocol::EquilibriumNone:
            CorrectionMethod = Equilibrium::NONE;
            break;
        case Protocol::EquilibriumReferenceController:
            ReferenceUpdate::ResetController();
            CorrectionMethod = Equilibrium::REFERENCE_UPDATE;
            break;
        case Protocol::EquilibriumPartialInformation:
            PartialInformation::ResetFilter();
            CorrectionMethod = Equilibrium::PARTIAL_INFORMATION;
            break;
        case Protocol::EquilibriumCurrentCorrection:
            CurrentCorrection::ResetController();
            CorrectionMethod = Equilibrium::CURRENT_CORRECTION;
            break;
        case Protocol::EngageParallelLoad:
            OutputLoadStep = true;
            break;
        case Protocol::DisengageParallelLoad:
            OutputLoadStep = false;
            break;
        case Protocol::EnableModeHopping:
            ModeHoppingEnabled = true;
            break;
        case Protocol::DisableModeHopping:
            ModeHoppingEnabled = false;
            break;
        default:
            break;
        }

        lastBufferIndex++;

        if (lastBufferIndex >= IPC_COMMAND_BUFFER_SIZE)
            lastBufferIndex = 0;

    }


    void ChangeConverter(ConverterID newConverter)
    {
        if(CurrentState!=OS_OFF)
            return;

        ConverterEnabled = false;

        activeConverter = newConverter;

        CurrentState = OS_CHANGING_CONVERTER_CONTROLLER;
    }


    void ChangeController(ControlStrategy newController)
    {
        if(CurrentState!=OS_OFF)
            return;

        ConverterEnabled = false;

        if (Controller::isClassicControl(newController))
        {
            Sensor::ConfigureFrequency(SWITCH_PWM_TBPRD);
            Switch::ConfigurePWM();
        }
        else if (Controller::isSwitchedControl(newController))
        {
            Sensor::ConfigureFrequency(SWITCH_SWITCHED_PWM_TBPRD);
            Switch::ConfigureGPIO();
        }

        controlStrategy = newController;

        CurrentState = OS_CHANGING_CONVERTER_CONTROLLER;
    }


    void CompleteConverterControllerChange(void)
    {
        if(CurrentState != OS_CHANGING_CONVERTER_CONTROLLER)
            return;

        CurrentState = OS_OFF;
    }


    void EnableOperation(void)
    {
        if(CurrentState!=OS_OFF)
            return;

        ReferenceUpdate::ResetController();
        PartialInformation::LoadFilter();
        CurrentCorrection::ResetController();
        VoltageController::ResetController();
        VoltageCurrentController::ResetController();



        if (activeConverter == ID_Boost)
        {
            CurrentState = OS_STARTING_PRE_LOAD;
            Relay::PreLoadCapacitor(true);
        }
        else
        {
            CurrentState = OS_RUNNING;
            Relay::PreLoadCapacitor(false);
        }

        if (Controller::isClassicControl(controlStrategy) && CurrentState==OS_RUNNING)
        {
            Switch::EnablePWM();
        }

        ConverterEnabled = true;
    }


    void DisableOperation(void)
    {
        if(CurrentState==OS_OFF)
            return;

        CurrentState = OS_OFF;

        Switch::DisablePWM();

        Relay::PreLoadCapacitor(false);

        ConverterEnabled = false;
    }


    void ContinuePreLoad(void)
    {
        if (    CurrentState != OS_STARTING_PRE_LOAD &&
                CurrentState != OS_PRE_LOAD &&
                CurrentState != OS_ENDING_PRE_LOAD)
            return;


        switch(CurrentState)
        {
        case OS_STARTING_PRE_LOAD:
            if (Relay::PreLoadCapacitor(true))
                CurrentState = OS_PRE_LOAD;

            if (Controller::isClassicControl(controlStrategy))
            {
                Switch::EnablePWM();
            }
            break;

        case OS_PRE_LOAD:
            if (*Vout >= *Vin * 0.9)
            {
                Relay::PreLoadCapacitor(false);
                CurrentState = OS_ENDING_PRE_LOAD;
            }
            break;

        case OS_ENDING_PRE_LOAD:
            if (!Relay::PreLoadCapacitor(false))
                CurrentState = OS_RUNNING;
            VoltageController::ResetController();
            VoltageCurrentController::ResetController();
            ReferenceUpdate::ResetController();
            PartialInformation::LoadFilter();
            CurrentCorrection::ResetController();
            break;
        }

    }


    bool SynchronousOperation(double IL)
    {
        if (ModeHoppingEnabled)
            return (IL > 0.1);
        else
            return true;
    }


    OperationState* GetCurrentState(void)
    {
        return &CurrentState;
    }
}
