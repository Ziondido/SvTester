/*
 * SVFunc.c
 *
 *  Created on: Apr 3, 2024c vdbf
 *      Author: Radga
 */

#include "SVFunc.h"
#include "math.h"


void SAMPLES_Get(ADC_HandleTypeDef hadc,SVT_SamplesValues *SamplesValuesPointer)
{
	for(int i=0;i<SAMPLES_TIMES;i++)
	{
		/* IMPORTANT: WAIT FOR THE CONVERSION TO COMPLETE BEFORE RETRIEVING THE VALUE
		* The reason is that HAL_ADC_GetValue does not wait and may retrieve an outdated sample
		* that has not been refreshed and saved in the ADC register.*/

		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);		//Waiting for ADC Conversion is Done
		SamplesValuesPointer->SamplesBuffer[i] = HAL_ADC_GetValue(&hadc);			//Get value from ADC
	}
}


/*
 * THE FUNCTION DO =>
 * CALBIRATION OF ALL SAMPLES IN SamplesBuffer ARRAY
 */
void SAMPLES_To_Voltage(SVT_SamplesValues *SamplesValuesPointer)
{
	for (int i = 0; i < SAMPLES_TIMES; i++)
	{
		SamplesValuesPointer->SamplesBuffer[i] 	*= 	3.3;
		SamplesValuesPointer->SamplesBuffer[i] 	/=	4096;
	}
}

/*
 * THE FUNCTION DO =>
 * SELF STM32 ADC CALIBRATION FOR REMOVE A DC OFFSET
 */
void SAMPLES_DC_Offset_Calibration(ADC_HandleTypeDef hadc)
{
	HAL_ADC_Stop(&hadc);
	HAL_Delay(50);
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
	HAL_ADC_Start(&hadc);
	HAL_Delay(50);
}


/*THE FUNCTION DO =>
 * SUM OF SamplesBuffer ARRAY VALUES */
void SAMPLES_Sum_Buffer(SVT_SamplesValues *SamplesValuesPointer)
{
	SamplesValuesPointer->SamplesSum = 0;
	for(int i=0;i<SAMPLES_TIMES;i++)
	{
		SamplesValuesPointer->SamplesSum += SamplesValuesPointer->SamplesBuffer[i];
	}
}


//the calibration fixed the range of the measured voltage. ie if the actual voltage range is 0 to 2, and
// the measured voltage is 0 to 1, then you have a gain error of 2. so you need to multiply all values by 2.
//look up gain error calibration, it is a function which compares the solpes of the measured range, and the actual range.
void SAMPLES_Gain_Error_Calibration(SVT_SamplesValues *SamplesValuesPointer)
{
	for(int i=0;i<SAMPLES_TIMES;i++)
	{
		SamplesValuesPointer->SamplesBuffer[i] *= GAIN_ERROR;
	}

}


/*
 * FUNCTION DESCRIPTION:
 * This function performs a DC (Direct Current) Signal Voltage Test by analyzing a buffer of ADC samples.
 * It checks for the presence of sustained voltage levels outside specified thresholds.
 * If the voltage consistently falls below the LowThreshold or rises above the HighThreshold, the test fails.
 * The function employs a fail counter to ensure sustained errors before declaring a test failure.
 *
 * PARAMETERS:
 * - LowThreshold: The lower acceptable threshold for the DC signal voltage.
 * - HighThreshold: The upper acceptable threshold for the DC signal voltage.
 *
 * RETURN:
 * - TEST_SUCCESS: The DC Signal Voltage Test is successful.
 * - TEST_FAILED: The test fails as the signal consistently falls outside the specified thresholds.
 *
 * IMPLEMENTATION DETAILS:
 * - The function iterates through a buffer of ADC samples, comparing each sample with the thresholds.
 * - If the signal consistently stays below the LowThreshold or above the HighThreshold for a specified count (90 samples in this case),
 *   the function returns TEST_FAILED.
 * - If the signal remains within the acceptable range throughout the iterations, the function returns TEST_SUCCESS.
 * - The SideOfError variable helps track whether the error is in the positive or negative voltage area for consecutive samples.
 * - Fail_Counter ensures a sustained error before triggering a failure.
 */
uint8_t DC_SV_Test(SVT_SamplesValues SamplesValues,float LowThreshold, float HighThreshold)
{
	uint8_t Fail_Counter = 0;
	SVT_ExceedingSide Exceeding = NONE_EXCEEDING;
	for(int i=0;i<SAMPLES_TIMES;i++)
	{
		if(SamplesValues.SamplesBuffer[i] < LowThreshold)
		{
			if(Exceeding == POSITIVE_AREA_EXCEEDING)
			{
				Fail_Counter = 0;
			}
			Exceeding = NEGATIVE_AREA_EXCEEDING;
			Fail_Counter++;
		}
		else if(SamplesValues.SamplesBuffer[i] > HighThreshold)
		{
			if(Exceeding == NEGATIVE_AREA_EXCEEDING)
			{
				Fail_Counter = 0;
			}
			Exceeding = POSITIVE_AREA_EXCEEDING;
			Fail_Counter++;
		}
		else
		{
			Fail_Counter = 0;
		}

		if(Fail_Counter >= 90)
		{
			return TEST_FAILED;
		}
	}
	return TEST_SUCCESS;
}

// all measured values, go through the function: value*10+1.6 IDEALY.
// in practice some errors apply(should be in the range of 1%).
uint8_t AC_RMS_Test(SVT_SamplesValues SamplesValues)
{
	SamplesValues.RMSValue = 0;
	float temp_calc=0;

	for(int i=0;i<SAMPLES_TIMES;i++)
	{
		temp_calc = SamplesValues.SamplesBuffer[i] - SELF_TEST_VREF;

		SamplesValues.RMSValue += temp_calc*temp_calc;
	}
	SamplesValues.RMSValue = sqrt(SamplesValues.RMSValue/SAMPLES_TIMES);
	if(SamplesValues.RMSValue<SV_RMS_THRESHOLD){
		return TEST_SUCCESS;
	}
	return TEST_FAILED;
}

void LEDs_Blink(uint8_t BlinkTimes,uint8_t BlinkTime)
{
	for(int i=0;i<BlinkTimes*2;i++)
	{
		HAL_GPIO_TogglePin(GPIOB, RED_LED_Pin);
		HAL_GPIO_TogglePin(GPIOB, GREEN_LED_Pin);
		HAL_Delay(BlinkTime);
	}

}
