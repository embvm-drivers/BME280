#include "fixed_point_math.h"
#include <stdint.h>

// Source: https://gist.github.com/Madsy/1088393

#define FP_FUNC_BASE 16

/* Computing the number of leading zeros in a word. */
static int32_t clz(uint32_t x)
{
	int32_t n;

	/* See "Hacker's Delight" book for more details */
	if(x == 0)
		return 32;
	n = 0;
	if(x <= 0x0000FFFF)
	{
		n = n + 16;
		x = x << 16;
	}
	if(x <= 0x00FFFFFF)
	{
		n = n + 8;
		x = x << 8;
	}
	if(x <= 0x0FFFFFFF)
	{
		n = n + 4;
		x = x << 4;
	}
	if(x <= 0x3FFFFFFF)
	{
		n = n + 2;
		x = x << 2;
	}
	if(x <= 0x7FFFFFFF)
	{
		n = n + 1;
	}

	return n;
}

int fp_ln(int val)
{
	int fracv = 0, intv, y, ysq, fracr, bitpos;
	/*
	fracv    -    initial fraction part from "val"
	intv    -    initial integer part from "val"
	y        -    (fracv-1)/(fracv+1)
	ysq        -    y*y
	fracr    -    ln(fracv)
	bitpos    -    integer part of log2(val)
	*/

	// const int ILN2 = 94548; /* 1/ln(2) with 2^16 as base*/
	const int ILOG2E = 45426; /* 1/log2(e) with 2^16 as base */

	const int ln_denoms[] = {
		(1 << FP_FUNC_BASE) / 1,  (1 << FP_FUNC_BASE) / 3,	(1 << FP_FUNC_BASE) / 5,
		(1 << FP_FUNC_BASE) / 7,  (1 << FP_FUNC_BASE) / 9,	(1 << FP_FUNC_BASE) / 11,
		(1 << FP_FUNC_BASE) / 13, (1 << FP_FUNC_BASE) / 15, (1 << FP_FUNC_BASE) / 17,
		(1 << FP_FUNC_BASE) / 19, (1 << FP_FUNC_BASE) / 21,
	};

	/* compute fracv and intv */
	bitpos = 15 - clz((uint32_t)val);
	if(bitpos >= 0)
	{
		++bitpos;
		fracv = val >> bitpos;
	}
	else if(bitpos < 0)
	{
		/* fracr = val / 2^-(bitpos) */
		++bitpos;
		fracv = val << (-bitpos);
	}

	// bitpos is the integer part of ln(val), but in log2, so we convert
	// ln(val) = log2(val) / log2(e)
	intv = bitpos * ILOG2E;

	// y = (ln_fraction_value−1)/(ln_fraction_value+1)
	y = ((int64_t)(fracv - (1 << FP_FUNC_BASE)) << FP_FUNC_BASE) / (fracv + (1 << FP_FUNC_BASE));

	ysq = (y * y) >> FP_FUNC_BASE;
	fracr = ln_denoms[10];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[9];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[8];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[7];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[6];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[5];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[4];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[3];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[2];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[1];
	fracr = (((int64_t)fracr * ysq) >> FP_FUNC_BASE) + ln_denoms[0];
	fracr = ((int64_t)fracr * (y << 1)) >> FP_FUNC_BASE;

	return intv + fracr;
}

int fp_exp(int val)
{
	int x;

	x = val;
	x = x - (((int64_t)x * (fp_ln(x) - val)) >> FP_FUNC_BASE);
	x = x - (((int64_t)x * (fp_ln(x) - val)) >> FP_FUNC_BASE);
	x = x - (((int64_t)x * (fp_ln(x) - val)) >> FP_FUNC_BASE);
	x = x - (((int64_t)x * (fp_ln(x) - val)) >> FP_FUNC_BASE);
	return x;
}

int fp_pow(int ebase, int exponent)
{
	return (fp_exp(((int64_t)exponent * fp_ln(ebase)) >> FP_FUNC_BASE));
}
