#include <src/Core/Switch/switch.h>

namespace Switch
{

    static short s_switch_state = 0;

    static short switch_to_power;

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

        s_switch_state = !s_switch_state;
        switch(s_switch_state)
        {
        case 0:
            GateDrive::SetState(GPIO_S1, false);
            switch_to_power = GPIO_S2;
            Timer::Switch_Start();
            break;
        case 1:
            GateDrive::SetState(GPIO_S2, false);
            switch_to_power = GPIO_S1;
            Timer::Switch_Start();
            break;
        default:
            GateDrive::SetState(GPIO_S1, false);
            GateDrive::SetState(GPIO_S2, false);
            break;

        }
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
