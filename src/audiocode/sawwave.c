#include "../audiodrivers/Audio_Drivers.h"
#include "../audiodrivers/cs43l22.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>



// Set up the sine look-up table:
#define PI 3.141592653589793 
#define SINESIZE 1024
#define PBSIZE 4096 

//angle range of 160 degrees divided by 11 notes - 2 octaves of pentatonic scale
#define PENTATONIC (160.0/11)

int16_t PlayBuff[PBSIZE]; 
int16_t SineBuff[SINESIZE]; 

float DELTA_T = 1.0/44100;// used in calculation for first order filter
float coefA = 870.297, coefB = 0.0034;// used in calculation for second order filter cutoff - coefficients




enum eNoteStatus { ready, going, finish } noteStatus = ready;
enum eBufferStatus { empty, finished, firstHalfReq, firstHalfDone,
			secondHalfReq, secondHalfDone } bufferStatus = empty; 
enum eMainStatus { normal, changingNoteOrBuffer } currentState = normal;

// Audio callbacks:
void myAudioHalfTransferCallback(void) {
	bufferStatus = firstHalfReq;
	
}

void myAudioTransferCompleteCallback(void) {
 myAudioChangeBuffer(PlayBuff, PBSIZE);
 bufferStatus = secondHalfReq;
}



void saw(float desiredFreq, float volume){

	for (int j = 0; j < SINESIZE; j++) {
    float sawtoothWave = 0.0;
		// harmonic iterations
    for (int harmonic = 1; harmonic <= 20; harmonic++) {
        float frequency = desiredFreq * harmonic; // get the harmonic frequency
        float amplitude = volume / harmonic; // Adjust the amplitude here

        sawtoothWave += amplitude * sin(j * 2.0 * PI * harmonic / SINESIZE);
    }

    SineBuff[j] = (int16_t)(sawtoothWave * 10000); //writng all values into array

	}

}



void coefficients(float Q, float ita, float* b0, float* b1, float* b2, float* a1, float* a2) {
		float denom =  1.0 + Q *ita + ita * ita; // ita refers to damping ratio
	
		*b0 = 1.0 / denom;
		*b1 = 2* (*b0);
		*b2 = *b0;
		*a1 = 2.0*(ita*ita - 1.0) * (*b0);
		*a2 = -(1.0 - Q*ita + ita*ita) * (*b0);

}

void audioinit(){
	// Initialise clock
	myAudioSpeedUpTheSystemClock();
	
	// Initialise the audio driver: 
	myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);

	// Initialise the audio buffer with silence: 
	for(int i=0; i <= PBSIZE; i++) {
		PlayBuff[i] = 0; 
	} 

}

void angleToFreq(float* f_0, float volume, float* phaseIncrement){
	int16_t angle = getangledata();
	//int angle = 8;
	//float noteRatio = 0.0;
	
	int noteRatio[11] = {-12, -10, -8, -5, -3, 0, 2, 4, 7, 9, 12};
	// for the angle in a range, have it set to a certain note based upon the initial f_0 value 
	int index = (angle + 80) / PENTATONIC;
	

	// calculate the ratio based upon the new note (2^(-12/12) to 2^(12/12))
	float ratio = pow(2, (noteRatio[index]/12));
	
	// multiply the old frequency by the new ratio
	float note = *f_0 * ratio; 
	

	
	// run new note through sawwave gen function
	saw(note, volume);
	// change value at phase increment to alter buffer size so the correct freq is output
	*phaseIncrement = (SINESIZE * note / ((float)AUDIO_FREQUENCY_44K));
}




void calcCutoff(float* desiredCutoff) {
	float distance = measureDistance();
  // with some constants, calc cutoff based upon range of 1-40cm mapping to 100-20kHz
	*desiredCutoff = coefA * pow(10, (coefB * distance));
}



// saw wave function
void sawwave(){
	// Set-up the phase and phase increment:
	float currentPhase = 0.0;
	
	float desiredFrequency = 440.0; // inital frequency pre rotation
	float volume = 1; // Adjust this value to control the overall volume
	float phaseIncrement = (SINESIZE * desiredFrequency / ((float)AUDIO_FREQUENCY_44K)); // increment for 
	
	  
	//Start the audio driver play routine
	myAudioStartPlaying(PlayBuff, PBSIZE); 
	// initialise values for output with filter
	float lastSampleInput = 0.0, lastLastSampleInput= 0.0, lastSampleOutput = 0.0, lastLastSampleOutput = 0.0, filteredSample = 0.0;
	
	// cutoff for filter

	float desiredCutoff = 0.0; // initialise cutoff
	calcCutoff(&desiredCutoff); // calculate cutoff from distance
	
	// coefficient initialisation
	float a1,a2,b0,b1,b2;
	// q factor initialisation
	float	Q = 1*0.5;

		
	 
	 
	angleToFreq(&desiredFrequency, volume, &phaseIncrement); // find angle and therefore frequency of note. 
	while(1) {
	
	
		// If there's been a request to fill half of the buffer,
		// then set the start and end points to fill:
		
		uint32_t startFill = 0, endFill = 0; // intialise buffer values
		if (bufferStatus == firstHalfReq) {
			startFill = 0;
			endFill = PBSIZE / 2;
			bufferStatus = firstHalfDone;
		} else if (bufferStatus == secondHalfReq) {
			startFill = PBSIZE / 2;
			endFill = PBSIZE;
			bufferStatus = secondHalfDone;
		}

			//float desiredCutoff = 0.0;
			

		if (startFill != endFill) {
			// calc values for coefficient calc
			calcCutoff(&desiredCutoff);
			float ff = desiredCutoff/AUDIO_FREQUENCY_44K;
			float ita = 1.0/ tan(PI*ff);
		
			// calc coefficients
			coefficients(Q, ita, &b0, &b1, &b2, &a1, &a2);
		
			// begin buffer fill loop
			for (int i = startFill; i < endFill; i += 2) {
			
				currentPhase += phaseIncrement;
				if (currentPhase > SINESIZE) currentPhase -= SINESIZE;
				int16_t nextSample = SineBuff[(uint16_t)(currentPhase)];
			
							
				// below is second order
				// using values stored from last sample, and coefficients which were calculated, sample is filtered
				float filteredSample = ( a1 * lastSampleOutput ) + ( a2 * lastLastSampleOutput ) + ( b0 * nextSample )  + ( b1 * lastSampleInput ) + ( b2 * lastLastSampleInput );
			

				PlayBuff[i] = (int16_t)(0.5*filteredSample); // volume decreased due to additions
				PlayBuff[i + 1] = (int16_t)(0.5*filteredSample);

			
				// set values for next time around
				lastLastSampleInput = lastSampleInput; 		// x[n-1] => x[n-2]
				lastSampleInput = nextSample;							// x[n] => x[n-1]
				lastLastSampleOutput = lastSampleOutput;	// y[n-1] => y[n-2] 
				lastSampleOutput = filteredSample;				// y[n] => y[n-1] or last output set
			
		
				
				coefficients(Q, ita, &b0, &b1, &b2, &a1, &a2);
			
		
			} 
		} // end of while loop 

	
	}
}
