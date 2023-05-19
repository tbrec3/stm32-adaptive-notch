/*
 * @file NotchFilter.c
 * @author Tobias Reck
 * @brief Module for implementing the adaptive notch filter.
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023 Tobias Reck
 *
 * This module contains all functions to use the notch filter struct
 * either as a normal notch filter or adaptively by using the
 * block filter function provided.
 */

#include <math.h>#include <notch_filter.h>


void NF_Init(NotchFilter *filt, double r, double fn, double fs) {
    // Update struct values
    filt->r = r;

	// Initialise the state buffers to zero
    for (uint32_t i=0; i<halfBUF_SIZE+2; i++) {
        filt->x[i] = 0;
        filt->y[i] = 0;
    }

    // Calculate the gain factor
    double w = 2 * M_PI * fn / fs;
    double an = 2 * cos(w);
    double delta = NF_GetDelta(an, r);

    // Update the Filter Coefficients
    filt->b[0] = delta;
    filt->b[1] = -an * delta;
    filt->b[2] = delta;

    filt->a[0] = 1;
    filt->a[1] = r * -an;
    filt->a[2] = r * r;
}

double NF_GetDelta(double an, double r) {
    // Gain Factor Delta to achieve Unity Gain
    double delta;

    /*  |H(z)| = (1 - an*z^-1 + z^-2) / (1 - r*an*z^-1 + r^2*z^-2)
     *  delta = 1 / |H(z)|
     *  Pick the Gain at w=0 -> z=+1 or w=pi -> z=-1
     *  based on the position of w0 to get
     *  the highest gain factor at the opposite position
     */
    if (an >= 0) {	// z = -1
        delta = (1 + r*an + r*r) / (1 + an + 1);
    } else {		// z = +1
        delta = (1 - r*an + r*r) / (1 - an + 1);
    }
    return delta;
}

void NF_Filter(NotchFilter *filt, uint32_t n) {
	void _transFunc(double *x, double *y, double *b, double *a, uint32_t n) {
		y[n] = b[0] * x[n] + b[1] * x[n-1] + b[2] * x[n-2] -
							 a[1] * y[n-1] - a[2] * y[n-2];
	}
	_transFunc(filt->x, filt->y, filt->b, filt->a, n);
}

void NF_UpdateCoeff(NotchFilter *filt, double an) {
    double delta = NF_GetDelta(an, filt->r);

    // Recalculate only the necessary Coefficients
    filt->b[0] = delta;
    filt->b[1] = -an * delta;
    filt->b[2] = delta;

    filt->a[1] = filt->r * -an;
}

void NF_SetNotchFreq(NotchFilter *filt, double fn, double fs) {
	// Calculate 'an' based on the notch frequency
    double w = 2 * M_PI * fn/fs;
    double an = 2 * cos(w);

    NF_UpdateCoeff(filt, an);
}

/***************** Adaptive Notch Functions *****************/

double ANF_Estimate(NotchFilter *filt, double an, uint32_t n, double lambda) {
	// Gradient Descent Algorithm
    double gradient = filt->y[n] * (filt->r * filt->y[n-1] - filt->x[n-1]);
    double new_an = an - lambda * gradient;

    // Range Clamp
    if (new_an < -2) new_an = -2;
    if (new_an >  2) new_an =  2;

    return new_an;
}

void ANF_BlockFilter_dbl(
		NotchFilter *filt,
		uint32_t *pSrc,
		uint32_t *pDst,
		uint32_t blockSize,
		uint8_t mode,
		double *an,
		double lambda)
{
	// Convert samples and write them into the input state buffer
	for (uint32_t i = 0; i < blockSize; i++) {
		filt->x[i+2] = ((double)(pSrc[i]) - OFFSET) / 2048;
	}

	// Keep track of current 'an' value
	// and take the last value from the block before
	double curr_an = an[blockSize-1];

	// Filter loop
	for (uint32_t i = 0; i < blockSize; i++) {
		// Filter the state buffers with the second position as output
		NF_Filter(filt, i+2);

		if (mode == ADAPT) {
			// Update 'an'
			curr_an = ANF_Estimate(filt, curr_an, i+2, lambda);
			an[i] = curr_an;
			NF_UpdateCoeff(filt, an[i]);
		}

		if (mode == FEED) {
			// Update coefficients using current 'an'
			NF_UpdateCoeff(filt, an[i]);
		}
	}

	// Copy last state values to the first position for the next block
	for (uint32_t i = 0; i < 2; i++) {
		filt->x[i] = filt->x[i + blockSize];
		filt->y[i] = filt->y[i + blockSize];
	}

	// Convert values from the state buffer and write them to the destination buffer
	for (uint32_t i = 0; i < blockSize; i++) {
		pDst[i] = (uint32_t)(filt->y[i+2]*2048 + OFFSET);
	}
}

