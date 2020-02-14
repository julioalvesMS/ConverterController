#include <src/Testes/testes.h>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_LEDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO LEDS 1-4 19-21  DA PLACA ligados aos GPIO 67,69,71,71     %
//OS LEDS PISCA 20 VEZES A CADA CHAMADA  DESTA FUN��O                                          %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_LEDS(void){
    int cont=0;
    int flag;
    while(cont<20){
        if(flag > 800000){
            LIGALED1;
            LIGALED2;
            //LIGALED3;
            //LIGALED4;
        }
        else{
            DESLIGALED1;
            DESLIGALED2;
            //DESLIGALED3;
            //DESLIGALED4;
        }

        if(flag >= 1600000){
            flag = 0;
            cont++;
        }

        flag++;
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_BOTOESS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 49 e 48, que est�o ligados aos bot�es 1 e 2               %
//Os LEDS 1,2 -19,18 ser�o ligados enquanto o bot�o BOT1 for pressionado                       %
//Os LEDS 3,4- 20,21 ser�o ligados enquanto o bot�o BOT2 for pressionado                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BOTOES(void){
    int cont=0;
    int flag;

    while(cont<20){
        if(BOT1==1){
            LIGALED1;
            //LIGALED3;
        }
        else{
          DESLIGALED1;
          //DESLIGALED3;
        }

        if(BOT2==1){
            LIGALED2;
            //LIGALED4;
        }
        else{
            DESLIGALED2;
            //DESLIGALED4;
        }

        if(flag >= 1600000){
            flag = 0;
            cont++;
        }
        flag++;
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_REL�S%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 23 e 16, que est�o ligados aos rel�s RELE1 E RELE2        %
//Os rel�s s�o ligados e desligados a cada 1 segundo                                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_RELES(void){
    int cont=0;
    int flag;

    while(cont<20){
        if(flag > 900000)     //9000 //11000
        {
            DESLIGARELE1;
            DESLIGARELE2;
        }
        else
        {
            LIGARELE1;
            LIGARELE2;
        }
        if(flag >= 1600000)     //9000 //11000
        {
            flag = 0;
            cont++;
        }

        flag++;
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_PWN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os canais os 12 canais ePWM                                                %
// OS PWM 1-6 s�o acionados com indice de modula��o (m) que varia de de-1 at�1                 %
// Caso o bot�o B1 seja acionado o Trip zone dos sensores � acionado                           %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_PWM(void)
{
    int cont=0;
    double m=-1;

    EALLOW;
    //Inicializa��o de pwm em zero
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZEINT.bit.OST = 1;

    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;

    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;

    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZEINT.bit.OST = 1;

    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZEINT.bit.OST = 1;

    EPwm6Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZEINT.bit.OST = 1;
    EDIS;

    while(cont<20)
    {
        if(m>1){
            m=-1;
            cont++;
        }
        else{
            m=m+0.000001;
        }

        //#######################################
        //  ATUALIZA PWM 1-3
        EPwm1Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm1A
        EPwm1Regs.CMPB.bit.CMPB = EPwm1Regs.CMPA.bit.CMPA;  //EPwm1B
        EPwm2Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm2A
        EPwm2Regs.CMPB.bit.CMPB = EPwm2Regs.CMPA.bit.CMPA;  //EPwm2B
        EPwm3Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm3A
        EPwm3Regs.CMPB.bit.CMPB = EPwm3Regs.CMPA.bit.CMPA;  //EPwm3B

        //#######################################
        //  ATUALIZA PWM4-6
        EPwm4Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm4A
        EPwm4Regs.CMPB.bit.CMPB = EPwm4Regs.CMPA.bit.CMPA;  //EPwm4B
        EPwm5Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm5A
        EPwm5Regs.CMPB.bit.CMPB = EPwm5Regs.CMPA.bit.CMPA;  //EPwm5B
        EPwm6Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm6A
        EPwm6Regs.CMPB.bit.CMPB = EPwm6Regs.CMPA.bit.CMPA;  //EPwm6B

        if(BOT1==1){
            LISGATZSENSORES;
        }
        else{
            DESLISGATZSENSORES;
        }
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS_RESET/BKR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa as sa�das de reset e de break dos VSCs                                     %
// Caso o bot�o B1 seja acionado as saidas RESETVSC(gpio62) E RESETVSC2 s�o acionadas          %
// Caso o bot�o B2 seja acionado as saidas BRK e BKR2 s�o acionadas                            %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_BKR_RST(void)
{
    int cont=0;
    int flag;

    while(cont<20)
    {
        if(BOT1==1){
            LIGARESETVSC;
            LIGARESETVSC2;
            LISGATZSENSORES;
        }
        else{
            DESLIGARESETVSC;
            DESLIGARESETVSC2;
            DESLISGATZSENSORES;
        }

        if(BOT2==1){
            LIGABKR;
            LIGABKR2;
        }
        else{
            DESLIGABKR;
            DESLIGABKR2;
        }

        if(flag >= 1600000) {
            flag = 0;
            cont++;
        }

        flag++;
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS ISOLADAS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 27,21,61,60 que controlam as SAIDAS ISOLADAS1-4           %
//AS SAIDAS PISCAM 20 VEZES A CADA CHAMADA  DESTA FUN��O                                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_SAIDAS_ISOLADAS(void)
{
    int cont=0;
    int flag;

    while(cont<20)
    {
        if(flag > 800000){
            LIGASAIDAISO1;
            LIGASAIDAISO2;
            LIGASAIDAISO3;
            LIGASAIDAISO4;
        }
        else{
            DESLIGASAIDAISO1;
            DESLIGASAIDAISO2;
            DESLIGASAIDAISO3;
            DESLIGASAIDAISO4;
        }
        if(flag >= 1600000){
            flag = 0;
            cont++;
        }
        flag++;
    }
}
