/*
 * @file NotchFilter.h
 * @author Tobias Reck
 * @brief Module for implementing the adaptive notch filter.
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023 Tobias Reck
 *
 */

#ifndef INC_NOTCH_FILTER_H_
#define INC_NOTCH_FILTER_H_
#include "stm32g4xx_hal.h"

/*
 * @brief Size of all signal buffers.
 *
 * @note Must be even for DMA to work!
 */
#define BUF_SIZE 10

/*
 * @brief Macro for better readability.
 */
#define halfBUF_SIZE BUF_SIZE/2

/*
 * @brief ADC/DAC offset value.
 *
 * This value is used to correctly normalize the values of the ADC
 * as well as for the DC offset in the output values for the DAC.
 * This is necessary due to the unipolar nature of both peripherals.
 */
#define OFFSET 2048

/*
 * @brief Modes for the block filter function.
 */
enum ANF_Mode{
	FIXED,
	FEED,
	ADAPT
};

/*
 * @brief Notch filter structure.
 *
 * @param x Input state buffer.
 * @param y Output state buffer.
 * @param b Numerator coefficients.
 * @param a Denominator coefficients.
 * @param r Magnitude of pole. [0, 1)
 */
typedef struct {
    double x[halfBUF_SIZE+2];
    double y[halfBUF_SIZE+2];
    double b[3];
    double a[3];
    double r;
} NotchFilter;

/*
 * @brief Initialises notch filter.
 *
 * @param filt Notch filter object.
 * @param r Magnitude of the pole. A number close to but smaller than
 * 			1 is recommended for smallest bandwidth.
 * @param fn Notch frequency in Hz.
 * @param fs Sampling frequency in Hz.
 */
void NF_Init(NotchFilter *filt, double r, double fn, double fs);

/*
 * @brief Helper Function to calculate the gain factor.
 *
 * @param an Determines the notch frequency. [-2, 2]
 * @param r Magnitude of the pole.
 *
 * @retval new value for delta
 */
double NF_GetDelta(double an, double r);

/*
 * @brief 	Function performing the filter operation and storing
 * 			the result in the output state buffer.
 *
 * @param filt Notch filter object.
 * @param n The position of the current sample in the state buffer.
 */
void NF_Filter(NotchFilter *filt, uint32_t n);

/*
 * @brief Updates the filter coefficients based on a new value for an.
 *
 * @param filt Notch filter object.
 * @param an Determines the notch frequency. [-2, 2]
 */
void NF_UpdateCoeff(NotchFilter *filt, double an);

/*
 * @brief Adjusts the filter parameter based on a given notch frequency.
 *
 * @param filt Notch filter object.
 * @param fn Notch frequency in Hz.
 * @param fs Sampling frequency in Hz.
 */
void NF_SetNotchFreq(NotchFilter *filt, double fn, double fs);

/*
 * @brief Performs the Gradient Descent Algorithm.
 *
 * @param filt Notch filter object.
 * @param an Determines the notch frequency. [-2, 2]
 * @param n The position of the current sample in the state buffer.
 * @param lambda 	The learning gain factor to control the step size.
 * 					A value of around 1 is recommended.
 *
 * @retval new estimate for an
 */
double ANF_Estimate(NotchFilter *filt, double an, uint32_t n, double lambda);

/*
 * @brief 	Filters a block of input samples and adapts
 * 			the parameter 'an' after each sample.
 *
 * @param filt 			Notch filter object used for the operation
 * @param pSrc 			Address of the input buffer
 * @param pDst 			Address of the output buffer
 * @param blockSize 	Specifies the number of elements to be processed
 * @param mode 			Sets the mode of the notch filter algorithm
 * 						This can be one of the following:
 * 	@arg FIXED: The parameters are updated once based on the first value in 'an'
 * 	@arg FEED:  The parameters are updated every sample based on 'an'
 * 	@arg ADAPT: The parameters are updated every sample using the
 * 				gradient descent algorithm
 * @param an		Points to a list of an values used for sharing between
 * 					notch filter objects. Necessary for modes FEED and ADAPT.
 * @param lambda 	The learning gain factor to control the step size.
 * 					A value of around 1 is recommended.
 */
void ANF_BlockFilter_dbl(
		NotchFilter *filt,
		uint32_t *pSrc,
		uint32_t *pDst,
		uint32_t blockSize,
		uint8_t mode,
		double *an,
		double lambda);

#endif /* INC_NOTCH_FILTER_H_ */
