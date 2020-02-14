//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// MEDIR VELOCIDADE														    <
// Est� fun��o mede a velocidade de rota��o a partir da leitura da posi��o 	<
// via encoder e da dire��o de rota��o										<
// medir_velocidade(POSI��O[graus], DIRE��O[1=ang crescente, 0 decrescente])<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//Prot�tipo das vari�veis que devem ser criadas no programa principal

extern float pos_2;				// Posi��o final
extern float pos_1;				// Posi��o da medida anterior
extern int divF;				// Divisor de frenquencia para medi��o
extern float vel_rpm;			// Velocidade medida em rpm



float medir_velocidade(float pos, float dir)
{
    float delta_pos;

	pos_2=pos;
	divF=divF+1;

	if (divF==100)
	{   						//C�lculo da velocidade a cada 100 itera��es
	    if(dir==1)
	    {      					// Ajuste delta_pos dire��o 1
	        if(pos_1>pos_2)
	        {
	            delta_pos=360-pos_1+pos_2;
	        }
	        else
	        {
				delta_pos=pos_2-pos_1;
			}
	    }
        else
        {           			// Ajuste delta_pos dire��o 2
            if(pos_2>pos_1)
            {
                delta_pos=-360-pos_1+pos_2;
            }
            else
            {
                delta_pos=pos_2-pos_1;
            }
        }
        vel_rpm=delta_pos*50;   // C�culo da velocidade, sinal indica sentido
        pos_1=pos_2;		   	//memoriza posi��o anterior p/ prox. medida
        divF=0; 				// reinicializa contador divisor de frequencia
	}

	return vel_rpm;
}
