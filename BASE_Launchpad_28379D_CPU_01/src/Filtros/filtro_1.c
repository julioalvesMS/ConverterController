double filtro_1(double vP)
{
    static double Bv[4]={0.000027213807988318872, 2*0.0000272138079883188725,0.000027213807988318872  };//50Hz em 30k
    static double Av[4]={1,-1.985190657896261,0.9852995131282144 };

	static double xv[4] = {0,0,0,0};
	static double yv[4] = {0,0,0,0};


	xv[0] = 1*vP; // adc voltage value is the current sample

	// calculate filter value
	yv[0] = Bv[0]*xv[0] + Bv[1]*xv[1] + Bv[2]*xv[2] - Av[1]*yv[1] - Av[2]*yv[2] ;//- A[3]*y[3];+ B[3]*x[3]


	// save past states so ready for next sample

	//x[3] = x[2];
	xv[2] = xv[1];
	xv[1] = xv[0];

	//y[3] = y[2];
	yv[2] = yv[1];
	yv[1] = yv[0];


	return yv[0];
}
