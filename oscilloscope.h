    /*
 * oscilloscope.h
 *
 *  Created on: Mar 26, 2019
 *      Author: mhle
 */

#ifndef OSCILLOSCOPE_H_
#define OSCILLOSCOPE_H_

void drawGrid(tContext sContext);
void getWaveform(int triggerDirection, uint16_t triggerVoltage);
void drawTriggerIcon(tContext sContext, uint16_t direction);
void drawWaveform(tContext sContext, uint16_t direction, uint16_t vScaleState, uint16_t samplingState, char* string, char* string2);
void drawSpectrum(tContext sContext, uint16_t samplingState);
void getSpectrum();
void ADC_ISR(void);
void ADCInit(void);


#endif /* OSCILLOSCOPE_H_ */
