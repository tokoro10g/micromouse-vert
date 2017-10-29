#include "mymath.h"

namespace MyMath {

	float sin(float x) {
		float sinVal, fract, in;                           /* Temporary variables for input, output */
		uint16_t index;                                        /* Index variable */
		float a, b;                                        /* Two nearest output values */
		int32_t n;
		float findex;

		// fit the range of the input
		if(x < 0.f) {
			x = 2.f*PI+x;
		}
		/* input x is in radians */
		/* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi */
		in = x * 0.159154943092f;

		/* Calculation of floor value of input */
		n = (int32_t) in;

		/* Make negative values towards -infinity */
		if(x < 0.0f)
		{
			n--;
		}

		/* Map input value to [0 1] */
		in = in - (float) n;

		/* Calculation of index of the table */
		findex = (float) FAST_MATH_TABLE_SIZE * in;
		index = ((uint16_t)findex) & 0x1ff;

		/* fractional value calculation */
		fract = findex - (float) index;

		/* Read two nearest values of input value from the sin table */
		a = sinTable_f32[index];
		b = sinTable_f32[index+1];

		/* Linear interpolation process */
		sinVal = (1.0f-fract)*a + fract*b;

		/* Return the output value */
		return (sinVal);
	}

	float cos(float x) {
		float cosVal, fract, in;                   /* Temporary variables for input, output */
		uint16_t index;                                /* Index variable */
		float a, b;                                /* Two nearest output values */
		int32_t n;
		float findex;

		// fit the range of the input
		if(x < 0.f) {
			x = 2.f*PI+x;
		}
		/* input x is in radians */
		/* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
		in = x * 0.159154943092f + 0.25f;

		/* Calculation of floor value of input */
		n = (int32_t) in;

		/* Make negative values towards -infinity */
		if(in < 0.0f)
		{
			n--;
		}

		/* Map input value to [0 1] */
		in = in - (float) n;

		/* Calculation of index of the table */
		findex = (float) FAST_MATH_TABLE_SIZE * in;
		index = ((uint16_t)findex) & 0x1ff;

		/* fractional value calculation */
		fract = findex - (float) index;

		/* Read two nearest values of input value from the cos table */
		a = sinTable_f32[index];
		b = sinTable_f32[index+1];

		/* Linear interpolation process */
		cosVal = (1.0f-fract)*a + fract*b;

		/* Return the output value */
		return (cosVal);
	}

	float fabs(float x) {
		if(x<0.f) x=-x;
		return x;
	}

	float sqrt(float x) {
		/*
		// x = [0 eeeeeeeeeee mmmmmmmmmmmmmmmmmmmmmmm]
		int *i = reinterpret_cast<int*>(&x);
		int j = (((*i/2-0x1fc00000)+0x3f800000)&0x3ff800000)+((*i&0x7fffff)/5*2);

		float *z = reinterpret_cast<float*>(&j);
		float y = (*z) * ((*i-0x3f800000)&(0x800000) ? 1.4142135623730951f : 1); // 2-3 sig figs of significance

		// two iterations of newton: y = y - (y^2 - x)/2y yields 2^-18 points of precision
		y = (y + x/y)/2; // this yields 2^-8 points of precision
		return (y + x/y)/2;
		*/
		float f;
		asm("VSQRT.F32 %0,%1" : "=t"(f) : "t"(x));
		return f;
	}

}
