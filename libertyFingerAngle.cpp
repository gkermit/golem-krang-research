//Use two liberty sensors to create two sets of lines, who's vectors can be used to calculate the angle between the finger and the palm.

#include <math.h>

double angle(double a1, double a2, double a3, double b1, double b2, double b3) {

	double theta = 0;
	
	//dot product of two lines
	dp = (a1)(b1) + (a2)(b2) + (a3)(a4);
		
	//magnitude of lines
	ma = sqrt(pow(a1,2) + pow(a2,2) + pow(a3,2));
	mb = sqrt(pow(b1,2) + pow(b2,2) + pow(b3,2));
	m = (ma)(mb);

	//calculation of angle
	theta = acos(dp/m);
	return theta;
}
