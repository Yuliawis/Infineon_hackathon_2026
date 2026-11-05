/* ========================================
 * utils.c
 * ========================================
 */

#include "utils.h"

// function to limit a value within a range
// constrain() ensures value stays within minVal and maxVal
int constrain_int(int x, int a, int b) 
{
  if (x < a) 
    return a;       // Return lower limit if x is below a
  else if (x > b) 
    return b;  // Return upper limit if x exceeds b
  else 
    return x;             // Return x if it's within the range
}

float constrain_float(float x, float a, float b) 
{
  if (x < a) 
    return a;       // Return lower limit if x is below a
  else if (x > b) 
    return b;  // Return upper limit if x exceeds b
  else 
    return x;             // Return x if it's within the range
}


// Re-maps a number from one range to another. That is, a value of fromLow 
// would get mapped to toLow, a value of fromHigh to toHigh, values 
// in-between to values in-between, etc.
// Does not constrain values to within the range, because out-of-range 
// values are sometimes intended and useful. The constrain() function may 
// be used either before or after this function, if limits to the ranges are
// desired.
// Note that the "lower bounds" of either range may be larger or smaller than
// the "upper bounds", so the map() function may be used to reverse a range of
// numbers, for example.
int map_int(int value, int in_min, int in_max, int out_min, int out_max) 
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_float(float value, float in_min, float in_max, float out_min, float out_max) 
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Integer square root rounded down
uint32_t sqrt_int(uint32_t value)
{
  uint32_t result = 0;
  uint32_t bit = 1UL << 30;

  while (bit > value)
  {
    bit >>= 2;
  }
  while (bit != 0)
  {
    if (value >= result + bit)
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }
  return result;
}

// CORDIC (vectoring mode) with a table of atan(2^-i) in millidegrees. Accuracy is about 2 millidegrees.
#define CORDIC_ITERATIONS (16)
#define CORDIC_MIN_MAGNITUDE (1L << 26)
static const int32_t cordic_atan_mdeg[CORDIC_ITERATIONS] =
{
  45000, 26565, 14036, 7125, 3576, 1790, 895, 448, 224, 112, 56, 28, 14, 7, 3, 2
};

int32_t atan2_mdeg(int32_t y, int32_t x)
{
  int32_t angle = 0;
  int32_t magnitude;

  if (x == 0 && y == 0)
  {
    return 0;
  }

  // CORDIC converges only in the right half plane, so rotate by 90 degrees first
  if (x < 0)
  {
    int32_t old_x = x;
    if (y >= 0)
    {
      x = y;
      y = -old_x;
      angle = 90000;
    }
    else
    {
      x = -y;
      y = old_x;
      angle = -90000;
    }
  }

  // Scale up for precision. Inputs must be below 2^26 in magnitude.
  magnitude = (x < 0 ? -x : x);
  if ((y < 0 ? -y : y) > magnitude)
  {
    magnitude = (y < 0 ? -y : y);
  }
  while (magnitude < CORDIC_MIN_MAGNITUDE)
  {
    x *= 2;
    y *= 2;
    magnitude *= 2;
  }

  for (int i = 0; i < CORDIC_ITERATIONS; ++i)
  {
    int32_t x_shifted = x >> i;
    int32_t y_shifted = y >> i;
    if (y > 0)
    {
      x += y_shifted;
      y -= x_shifted;
      angle += cordic_atan_mdeg[i];
    }
    else
    {
      x -= y_shifted;
      y += x_shifted;
      angle -= cordic_atan_mdeg[i];
    }
  }
  return angle;
}


/* [] END OF FILE */
