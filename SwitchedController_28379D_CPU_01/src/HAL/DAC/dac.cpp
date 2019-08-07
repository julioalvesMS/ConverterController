#include <src/HAL/DAC/dac.h>

namespace DAC_HAL
{
    volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};

    void Configure(void)
    {
        EALLOW;
        DAC_PTR[DAC_NUM_IL]->DACCTL.bit.DACREFSEL = REFERENCE;
        DAC_PTR[DAC_NUM_IL]->DACOUTEN.bit.DACOUTEN = 1;
        DAC_PTR[DAC_NUM_IL]->DACVALS.all = 0;
        DELAY_US(10); // Delay for buffered DAC to power up

        DAC_PTR[DAC_NUM_VOUT]->DACCTL.bit.DACREFSEL = REFERENCE;
        DAC_PTR[DAC_NUM_VOUT]->DACOUTEN.bit.DACOUTEN = 1;
        DAC_PTR[DAC_NUM_VOUT]->DACVALS.all = 0;
        DELAY_US(10); // Delay for buffered DAC to power up
        EDIS;
    }

    void Update(unsigned int IL, unsigned int VOUT)
    {
        DAC_PTR[DAC_NUM_IL]->DACVALS.all = IL;
        DAC_PTR[DAC_NUM_VOUT]->DACVALS.all = VOUT;
    }
}
