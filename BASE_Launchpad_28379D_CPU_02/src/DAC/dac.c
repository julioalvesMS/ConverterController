#include <src/DAC/dac.h>

void DAC_Configure(void)
{
    spi_fifo_init();    // Configura SPI-FIFO DA DAC
    spi_init();         // Inicializa SPI-FIFO DA DAC
}

void DAC_SendData(Uint16 opt)
{
    switch(opt)
    {
    case 0:
        enviar_dac_spi_4Canais(Ix,Iy,Iz,Icc2);
        break;
    case 1:
        enviar_dac_spi_4Canais(Vx,Vy,Vz,Vcc2);
        break;
    case 2:
        enviar_dac_spi_4Canais(Iu,Iv,Iw,Icc1);
        break;
    case 3:
        enviar_dac_spi_4Canais(Vu,Vv,Vw,Vcc1);
        break;
    default:
        break;
    }
}
