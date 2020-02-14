float filtro_2(float vP)
{

	//static float Bv[4]={0.00952576, 2*0.00952576, 0.00952576};//1000Hz em 30k
	//static float Av[4]={1,  -1.70555214,   0.743655195};

	//static float Bv[4]={0.0025505351585362935, 2*0.0025505351585362935,0.0025505351585362935  };//500Hz em 30k
	//static float Av[4]={1,-1.8521464853959357, 0.862348626030081};
//
    static float Bv[4]={0.000027213807988318872, 2*0.0000272138079883188725,0.000027213807988318872  };//50Hz em 30k
    static float Av[4]={1,-1.985190657896261,0.9852995131282144 };

	//static float Bv[4]={0.1550510257216822, 2*0.1550510257216822, 0.1550510257216822        };//10000Hz em 30k
	//static float Av[4]={1,-0.62020410288672889, 0.24040820577345759};

//	static float Bv[4]={0.000241359049042, 2*0.000241359049042, 0.000241359049042};//1000Hz em 20k
//	static float Av[4]={1,  -1.955578240315,   0.9565436765112};

	static float xv[4] = {0,0,0,0};
	static float yv[4] = {0,0,0,0};


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

//
}
