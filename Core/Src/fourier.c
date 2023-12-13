
#include <math.h>
#include "fourier.h"
#include "main.h"
#include <complex.h>

static uint16_t FFT_LENGTH;
static uint16_t FFT_LENGTH_BIT;

static uint16_t bit_reverse(uint16_t x)
{
	uint16_t mask = FFT_LENGTH_BIT - 1;
	uint16_t num = 0;

	while (x) {
		num |= ((x % 2) << mask);
		mask--;
		x /= 2;
	}

	return num;
}

void array_reverse(float array[])
{
	uint16_t i, j;
	float temp;

	for (i = 0; i < FFT_LENGTH; i++) {
		j = bit_reverse(i);
		if (i < j) {
			temp = array[i];
			array[i] = array[j];
			array[j] = temp;
		}
	}
}

void FFT_Compute(float data[], uint16_t N)
{
	FFT_LENGTH = N;
	FFT_LENGTH_BIT = log2(FFT_LENGTH);

	complex float out_fft[FFT_LENGTH];
	complex float last_fft[FFT_LENGTH];
	complex float W[FFT_LENGTH];

	array_reverse(data);

	for (uint16_t i = 0; i < FFT_LENGTH; i += 2)
		last_fft[i] = data[i] + data[i+1];
	for (uint16_t i = 1; i < FFT_LENGTH; i += 2)
		last_fft[i] = data[i-1] - data[i];

	uint16_t fft_half = FFT_LENGTH/2;
	uint16_t W_offset, W_index;
	uint16_t counter;
	int16_t  index_offset_a, index_offset_b, fft_stage_value;

	for (uint16_t i = 0; i < FFT_LENGTH; i++) {
		if (i == 0)
			W[i] = 1;
		else if (i == fft_half)
			W[i] = -1;
		else
			W[i] = cos((2 * M_PI * i) / FFT_LENGTH) - I*sin((2 * M_PI * i) / FFT_LENGTH);
	}

	for (uint16_t i = 1; i < FFT_LENGTH_BIT; i++) {
		index_offset_b = fft_stage_value = (1 << i);
		index_offset_a = 0;
		counter = 0;
		W_offset = bit_reverse(fft_stage_value);
		W_index = 0;
		for (uint16_t j = 0; j < FFT_LENGTH; j++) {
			if (W_index == 0)
				out_fft[j] = last_fft[j + index_offset_a] + last_fft[j + index_offset_b];
			else if (W_index == fft_half)
				out_fft[j] = last_fft[j + index_offset_a] - last_fft[j + index_offset_b];
			else
				out_fft[j] = last_fft[j + index_offset_a] + (last_fft[j + index_offset_b] * W[W_index]);

			counter++;
			if (counter == fft_stage_value) {
				counter = 0;
				index_offset_a = (index_offset_a == 0 ? -fft_stage_value : 0);
				index_offset_b = (index_offset_b == fft_stage_value ? 0 : fft_stage_value);
			}

			W_index += W_offset;
			if (W_index == FFT_LENGTH)
				W_index = 0;
		}
		for (uint16_t j = 0; j < FFT_LENGTH; j++)
			last_fft[j] = out_fft[j];
	}

	for (uint16_t i = 0; i < FFT_LENGTH; i++)
		data[i] = cabs(out_fft[i]);
}

