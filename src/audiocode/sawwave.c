#include "../audiodrivers/Audio_Drivers.h"
#include "../audiodrivers/cs43l22.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>


// Set up the sine look-up table:
#define PI 3.141592653589793 
#define SINESIZE 1024
#define PBSIZE 4096 


int16_t PlayBuff[PBSIZE]; 
int16_t SineBuff[SINESIZE]; 
float DELTA_T = 2.0/44100;


enum eNoteStatus { ready, going, finish } noteStatus = ready;
enum eBufferStatus { empty, finished, firstHalfReq, firstHalfDone,
secondHalfReq, secondHalfDone } bufferStatus = empty; 

// Audio callbacks:
void myAudioHalfTransferCallback(void) {
	bufferStatus = firstHalfReq;
}

void myAudioTransferCompleteCallback(void) {
 myAudioChangeBuffer(PlayBuff, PBSIZE);
 bufferStatus = secondHalfReq;
} 


void sawwave(){

	myAudioSpeedUpTheSystemClock();
	
	// Initialise the audio driver: 
	myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);

	// Initialise the audio buffer with silence: 
	for(int i=0; i <= PBSIZE; i++) {
		PlayBuff[i] = 0; 
	} 
	
	// Set-up the phase and phase increment:
	float currentPhase = 0.0;
	float desiredFreq = 440.0;
	float phaseIncrement = (SINESIZE * desiredFreq / ((float)AUDIO_FREQUENCY_44K));
	

	
	float volume = 1; // Adjust this value to control the overall volume

	for (int j = 0; j < SINESIZE; j++) {
    float sawtoothWave = 0.0;

    for (int harmonic = 1; harmonic <= 10; harmonic++) {
        float frequency = desiredFreq * harmonic;
        float amplitude = volume / harmonic; // Adjust the amplitude here

        sawtoothWave += amplitude * sin(j * 2.0 * PI * harmonic / SINESIZE);
    }

    SineBuff[j] = (int16_t)(sawtoothWave * 10000);
	}
	
	
	
	
	                                   
	/*for (int i = 0; i < SINESIZE; i++) {
		float q = 32760 * sin(i * 2.0 * PI / SINESIZE);
		SineBuff[i] = (int16_t)q;
		}*/
	
	//Start the audio driver play routine
	myAudioStartPlaying(PlayBuff, PBSIZE); 
	
	/*while(1){
		if (bufferStatus == firstHalfReq) {
			for (int i=0; i<=PBSIZE/2 -1;++1){
				PlayBuff[i] = 0;
			}
			firstHalfDone();
		}
		else if (bufferStatus = secondHalfReq){
			for (int i=PBSIZE/2; i<=PBSIZE-1;++1){
				PlayBuff[i] = 0;
			}
			secondHalfDone();
		
		}*/
		
		
		float lastSampleInput = 0.0, lastSampleOutput = 0.0;
		//float a1 = 0.9776, b0 =  0.0112;
		
		float desiredCutoff = 500.0 * (2*PI);
	
	
		
		
		float a1 = -( ( desiredCutoff - ( 2/DELTA_T ) ) / ( desiredCutoff + (2/DELTA_T) ) );
		float b0 = desiredCutoff / (desiredCutoff+2/DELTA_T);
		
		
	 while (1){
		// If there's been a request to fill half of the buffer,
		// then set the start and end points to fill:
		 
		uint32_t startFill = 0, endFill = 0;
		if (bufferStatus == firstHalfReq) {
			startFill = 0;
			endFill = PBSIZE / 2;
			bufferStatus = firstHalfDone;
}
		
		else if (bufferStatus == secondHalfReq) {
			startFill = PBSIZE / 2;
			endFill = PBSIZE;
			bufferStatus = secondHalfDone;
 }

		if (startFill != endFill) {
			for (int i = startFill; i < endFill; i += 2) {
				currentPhase += phaseIncrement;
				if (currentPhase > SINESIZE) currentPhase -= SINESIZE;
				int16_t nextSample = SineBuff[(uint16_t)(currentPhase)];
				
				float filteredSample = a1 * lastSampleOutput + b0 * (nextSample + lastSampleInput);				
				
				PlayBuff[i] = (int16_t)(0.5*filteredSample);
				PlayBuff[i + 1] = (int16_t)(0.5*filteredSample);
				
				lastSampleInput = nextSample;
				lastSampleOutput = filteredSample;
			}
			
		}
	} // end of while loop 


}