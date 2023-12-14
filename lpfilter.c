#include <math.h>
#include <stdlib.h>
// #include <cmath>
#define PI 3.141592653589793238462643383279502884197169399375105820974944 

float b0, b1, b2, a1, a2, norm, K;


 

/* // taken from biquad calc
float calcBiquad(Fc, Fs, Q, peakGain) {
	
	// Fc is cutoff, Fs is sample rate
	
	//V = pow(10.0, abs(peakGain) / 20); <= used for other filters
	K = tan(PI * Fc / Fs);
	//switch (type) {
		//case 'lowpass':
	norm = 1.0 / (1 + K/Q + K*K);
	b0 = K * K * norm;
	b1 = 2 * b0;
	b2 = b0;
	a1 = 2 * (K * K - 1) * norm;
	a2 = (1- K/Q + K*K);
			//break;
	//} // can have other cases
	return (b0, b1, b2, a1, a2);
}
*/	




int lpfilter (Fs, Fc, lastSampleInput, lastLastSampleInput, lastSampleOutput, lastLastSampleOutput, nextSample) {
	
	double ff = Fc/Fs;
	double ita = 1.0/ tan(PI*ff);
	double Q = 1*0.5;
	b0 = 1.0 / (1.0 + Q*ita + ita*ita);
	b1 = 2*b0;
	b2 = b0;
	a1 = 2.0*(ita*ita - 1.0) * b0;
	a2 = -(1.0 - Q*ita + ita*ita) * b0;
	
	//calcBiquad(Fc, Fs, Q); 
	
	
	return ((1)* a1 * lastSampleOutput ) + ((1)* a2 * lastLastSampleOutput ) + ( b0 * nextSample )  + ( b1 * lastSampleInput ) + ( b2 * lastLastSampleInput );
	
}