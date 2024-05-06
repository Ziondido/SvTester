/*
 * SVFunc.h
 *
 *  Created on: Apr 3, 2024
 *      Author: Radga
 */

#ifndef SRC_SVFUNC_H_
#define SRC_SVFUNC_H_

#include "main.h"

#define SAMPLES_TIMES 900				// The number of samples that the tester makes

#define GAIN_ERROR 	1.0125944037  		// Calculated value

#define SELF_TEST_ADC_VAL	1.88460000 	// IN VOLTS	, AUX EN - RESET , SW7 - RESET
#define SELF_TEST_VREF		1.60100000	// IN VOLTS , AUX EN - RESET , SW7 - SET

#define SV_DC_THRESHOLD 1.05			// The DC threshold for sv test
#define SV_RMS_THRESHOLD 0.85			// The RMS threshold for sv test

// Enum for test status
typedef enum
{
	TEST_FAILED		= 0x00U,
	TEST_SUCCESS	= 0x01U
} SVT_TestStatus;

// Enum for operation modes
typedef enum
{
	SLEEP_MODE		= 0x00U,
	SELF_TEST		= 0x01U,
	SV_TEST			= 0x02U
} SVT_Mode;

// Enum for exceeding side pointer
typedef enum
{
	NONE_EXCEEDING			= 0x00U,
	POSITIVE_AREA_EXCEEDING	= 0x01U,
	NEGATIVE_AREA_EXCEEDING	= 0x02U
} SVT_ExceedingSide;

// this structure for storing all samples values
typedef struct SVT_SamplesValues
{
	float SamplesBuffer[SAMPLES_TIMES];
	float SamplesSum;
	float SamplesAvg;
	float RMSValue;
} SVT_SamplesValues;


void SAMPLES_Get(ADC_HandleTypeDef hadc,SVT_SamplesValues *SamplesValuesPointer);

void SAMPLES_To_Voltage(SVT_SamplesValues *SamplesValuesPointer);

void SAMPLES_DC_Offset_Calibration(ADC_HandleTypeDef hadc);

void SAMPLES_Sum_Buffer(SVT_SamplesValues *SamplesValuesPointer);

void SAMPLES_Gain_Error_Calibration(SVT_SamplesValues *SamplesValuesPointer);

uint8_t DC_SV_Test(SVT_SamplesValues SamplesValues,float LowThreshold, float HighThreshold);

uint8_t AC_RMS_Test(SVT_SamplesValues SamplesValues);

void LEDs_Blink(uint8_t BlinkTimes,uint8_t BlinkTime);

#endif /* SRC_SVFUNC_H_ */
