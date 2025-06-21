/**
 * @file        fuzzy.c
 * @copyright
 * @license
 * @version     0.0.0
 * @date
 * @author      Khoi Nguyen Thanh
 * @brief       none
 *
 * @note        none
 *
 * @example     none
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */

/* Includes ----------------------------------------------------------- */
#include "fuzzy.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
float fuzzy_trapezium(float data, float left, float center_left, float center_right, float right)
{
  if ((left == center_left) && (center_right != right))
  {
    if ((data >= left) && (data <= center_right))
    {
      return 1;
    }
    else if ((data >= center_right) && (data < right))
    {
      return ((right - data) / (right - center_right));
    }
    else // data < left, data >= right
    {
      return 0;
    }
  }
  else if ((center_right == right) && (left != center_left))
  {
    if ((data >= center_left) && (data <= right))
    {
      return 1;
    }
    else if ((data > left) && (data <= center_left))
    {
      return ((data - left) / (center_left - left));
    }
    else // data < left
    {
      return 0;
    }
  }
  else if ((left == center_left) && (center_right == right))
  {
    if ((data >= left) && (data <= right))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  if ((data < left) || (data >= right))
  {
    return 0;
  }
  else if ((data >= left) && (data < center_left))
  {
    return ((data - left) / (center_left - left));
  }
  else if ((data >= center_left) && (data < center_right))
  {
    return 1;
  }
  else if ((data >= center_right) && (data < right))
  {
    return ((right - data) / (right - center_right));
  }
  return 0;
}

float fuzzy_triangle(float data, float left, float center, float right)
{
  if ((data < left) || (data >= right))
  {
    return 0;
  }
  else if ((data >= left) && (data < center))
  {
    return ((data - left) / (center - left));
  }
  else if ((data >= center) && (data < right))
  {
    return ((right - data) / (right - center));
  }
  return 0;
}

float fuzzy_min(float a, float b)
{
  if (a < b)
  {
    return a;
  }
  return b;
}

float fuzzy_min_4_input(float a, float b, float c, float d)
{
  float min_value = a;
  if (b < min_value)
    min_value = b;
  if (c < min_value)
    min_value = c;
  if (d < min_value)
    min_value = d;
  return min_value;
}

float fuzzy_min_3_input(float a, float b, float c)
{
  float min_value = a;
  if (b < min_value)
    min_value = b;
  if (c < min_value)
    min_value = c;
  return min_value;
}

float fuzzy_saturation(float data)
{
  if (data >= 1)
  {
    data = 1;
  }
  else if (data <= -1)
  {
    data = -1;
  }
  return data;
}

float fuzzy_normalize(float data, float coefficient)
{
  data = data / coefficient;
  return fuzzy_saturation(data);
}

float fuzzy_denormalize(float data, float coefficient)
{
  return (data * coefficient);
}

float fuzzy_sum_2d(float *data, uint16_t row_len, uint16_t column_len)
{
  float sum = 0;
  for (int row = 0; row < row_len; row++)
  {
    for (int column = 0; column < column_len; column++)
    {
      sum += *(data + (row * column_len) + column);
    }
  }
  return sum;
}

float fuzzy_weight_sum(float *data, float y, uint16_t len)
{
  float sum = 0;
  for (int i = 0; i < len; i++)
  {
    sum += (*(data + i));
  }
  sum = sum * y;
  return sum;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
