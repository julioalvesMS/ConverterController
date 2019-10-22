#include <src/OperationManagement/manager.h>

extern double *Vin, *Vout;
extern bool ConverterEnabled;
extern ConverterID activeConverter;

namespace Manager
{
    OperationState CurrentState = OS_OFF;


    void ChangeConverter(ConverterID newConverter)
    {
        if(CurrentState!=OS_OFF)
            return;

        ConverterEnabled = false;

        activeConverter = newConverter;

        CurrentState = OS_CHANGING_CONVERTER;
    }


    void CompleteConverterChange(void)
    {
        if(CurrentState != OS_CHANGING_CONVERTER)
            return;

        CurrentState = OS_OFF;
    }


    void EnableOperation()
    {
        if(CurrentState!=OS_OFF)
            return;

        Equilibrium::ResetController();

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

        ConverterEnabled = true;
    }


    void DisableOperation()
    {
        if(CurrentState!=OS_RUNNING)
            return;

        CurrentState = OS_OFF;

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
            break;
        }

    }


    OperationState* GetCurrentState(void)
    {
        return &CurrentState;
    }
}
