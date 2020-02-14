//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// MEDIR VELOCIDADE														    <
// Está função mede a velocidade de rotação a partir da leitura da posição 	<
// via encoder e da direção de rotação										<
// medir_velocidade(POSIÇÃO[graus], DIREÇÃO[1=ang crescente, 0 decrescente])<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//Protótipo das variáveis que devem ser criadas no programa principal

extern float pos_2;				// Posição final
extern float pos_1;				// Posição da medida anterior
extern int divF;				// Divisor de frenquencia para medição
extern float vel_rpm;			// Velocidade medida em rpm



float medir_velocidade(float pos, float dir)
{
    float delta_pos;

	pos_2=pos;
	divF=divF+1;

	if (divF==100)
	{   						//Cálculo da velocidade a cada 100 iterações
	    if(dir==1)
	    {      					// Ajuste delta_pos direção 1
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
        {           			// Ajuste delta_pos direção 2
            if(pos_2>pos_1)
            {
                delta_pos=-360-pos_1+pos_2;
            }
            else
            {
                delta_pos=pos_2-pos_1;
            }
        }
        vel_rpm=delta_pos*50;   // Cáculo da velocidade, sinal indica sentido
        pos_1=pos_2;		   	//memoriza posição anterior p/ prox. medida
        divF=0; 				// reinicializa contador divisor de frequencia
	}

	return vel_rpm;
}
