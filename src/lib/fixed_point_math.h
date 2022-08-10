#ifndef FIXED_POINT_MATH_H_
#define FIXED_POINT_MATH_H_

/// Functionality to support fixed point math.
/// These functions expect numbers in the format Q16

int fp_ln(int val);
int fp_exp(int val);
int fp_pow(int ebase, int exponent);

#endif // FIXED_POINT_MATH_H_
