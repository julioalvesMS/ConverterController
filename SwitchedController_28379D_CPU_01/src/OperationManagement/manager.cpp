#include <src/OperationManagement/manager.h>

extern double *Vin, *Vout;
extern bool ConverterEnabled;
extern ConverterID activeConverter;
extern ControlStrategy controlStrategy;

namespace Manager
{
    OperationState CurrentState = OS_OFF;


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
            Sensor::ConfigureFrequency(125);
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

        Equilibrium::ResetController();
        PID::ResetController();



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
        if(CurrentState!=OS_RUNNING)
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
            if (*Vout >= *Vin * 0.95)
            {
                Relay::PreLoadCapacitor(false);
                CurrentState = OS_ENDING_PRE_LOAD;
            }
            break;

        case OS_ENDING_PRE_LOAD:
            if (!Relay::PreLoadCapacitor(false))
                CurrentState = OS_RUNNING;
            PID::ResetController();
            break;
        }

    }


    OperationState* GetCurrentState(void)
    {
        return &CurrentState;
    }
}
