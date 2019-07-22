#include <src/Core/Switch/switch.h>

namespace Switch
{

    static short s_switch_state = 0;

    static short switch_to_power;

    static int temp = 0;

    void Configure()
    {
        GateDrive::Configure();

        EALLOW;  // This is needed to write to EALLOW protected registers
        PieVectTable.TIMER2_INT = &cpu_timer2_isr;
        EDIS;    // This is needed to disable write to EALLOW protected registers
    }

    void SetState(short state)
    {
        //
        // If the switch is already ate the desired state, do nothing
        //
//        if (state == s_switch_state)
//            return;

        //
        // Toggle switch and register new state
        //
//        s_switch_state = state;

        temp++;

        s_switch_state = temp < 3;


        switch(s_switch_state)
        {
        case 0:
            TurnOff(GPIO_S1);
            TurnOn(GPIO_S2);
            break;
        case 1:
            TurnOff(GPIO_S2);
            TurnOn(GPIO_S1);
            break;
        default:
            TurnOff(GPIO_S1);
            TurnOff(GPIO_S2);
            break;

        }

        temp = temp % 3;
    }

    void TurnOff(short gate)
    {
        GateDrive::SetState(gate, false);
    }

    void TurnOn(short gate)
    {
        switch_to_power = gate;
        Timer::Switch_Start();
    }


    //
    // cpu_timer0_isr - CPU Timer0 ISR with interrupt counter
    //
    __interrupt void cpu_timer2_isr(void)
    {
       CpuTimer2.InterruptCount++;

       GateDrive::SetState(switch_to_power, true);

       Timer::Switch_Stop();
    }

}
