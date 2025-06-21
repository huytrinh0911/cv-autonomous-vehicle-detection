/**
 * @file       bsp_utils.c
 * @copyright
 * @license    This project is released under the Fiot License.
 * @version    0.0.0
 * @date
 * @author
 * @author
 *
 * @brief
 *
 * @note
 * @example
 *
 * @example
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_utils.h"
#include "tim.h"

/* Private defines ---------------------------------------------------- */
#define US2MS 		 	1000   		// number of usec in a msec

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static TIM_HandleTypeDef *htim_delay = &htim5;  // APB1 - 84 MHz
static void (*callback[NUM_DELAY_ENTRIES])(void);

static const uint32_t tim_channel[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};


/* Private function prototypes ---------------------------------------- */
void bsp_delay_init(void);
void delay_timer_callback(TIM_HandleTypeDef *htim);

/* Function definitions ----------------------------------------------- */
void bsp_utils_init(void)
{
    bsp_delay_init();
}

int8_t delay_us(uint32_t us, void *new_callback)
{
	us += 1; // To make sure that the delay is at least 1us when `us` = 1

	// TODO: How to handle case where we set a callback whose delay is very soon and its match will be missed
	// TODO: Handle wrap-around
	if (NULL == new_callback) // Blocking delay
	{
		uint32_t match = __HAL_TIM_GET_COUNTER(htim_delay) + us;
		while (__HAL_TIM_GET_COUNTER(htim_delay) < match)
			;
	}
	else // Non-blocking delay
	{
		// Stop the timer
		HAL_TIM_OC_Stop_IT(htim_delay, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop_IT(htim_delay, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop_IT(htim_delay, TIM_CHANNEL_3);
		HAL_TIM_OC_Stop_IT(htim_delay, TIM_CHANNEL_4);

		int tim_index;
		int duplicate_pos = -1, empty_pos = -1;

		// Get the next available timer channel
		for (tim_index = 0; tim_index < NUM_DELAY_ENTRIES; tim_index++)
		{
			if (callback[tim_index] == new_callback)
			{
				duplicate_pos = tim_index;
			}
			if (callback[tim_index] == NULL && empty_pos == -1)
			{
				empty_pos = tim_index;
			}
		}
		tim_index = (duplicate_pos == -1) ? empty_pos : duplicate_pos;

		if (tim_index == -1)
		{
			return -1; // No available channel
		}

		// Update channel's callback function
		callback[tim_index] = new_callback;

		// Update Capture Compare register and restart timer channels
		__HAL_TIM_DISABLE(htim_delay);

        uint32_t cnt_remain = 0xFFFFFFFF - __HAL_TIM_GET_COUNTER(htim_delay);
        if(cnt_remain > us)
        {
            __HAL_TIM_SET_COMPARE(htim_delay, tim_channel[tim_index], __HAL_TIM_GET_COUNTER(htim_delay) + us);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(htim_delay, tim_channel[tim_index], us - cnt_remain);
        }

		__HAL_TIM_ENABLE(htim_delay);
		HAL_TIM_OC_Start_IT(htim_delay, TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(htim_delay, TIM_CHANNEL_2);
		HAL_TIM_OC_Start_IT(htim_delay, TIM_CHANNEL_3);
		HAL_TIM_OC_Start_IT(htim_delay, TIM_CHANNEL_4);

		return tim_index;
	}

	return -1;
}

int8_t delay_ms(uint32_t ms, void *new_callback)
{
    return delay_us(ms * US2MS, new_callback); 
}

void delay_cancel(void *old_callback)
{
	for (int i = 0; i < NUM_DELAY_ENTRIES; i++)
	{
		if (callback[i] == old_callback)
		{
			callback[i] = NULL;
			__HAL_TIM_SET_COMPARE(htim_delay, tim_channel[i], 0);
			// return;
		}
	}
}

/* Private definitions ----------------------------------------------- */
void bsp_delay_init(void)
{
  __HAL_TIM_SET_PRESCALER(htim_delay, 84 - 1);

  // Clear out delay table
  for (int i = 0; i < NUM_DELAY_ENTRIES; i++)
  {
    callback[i] = NULL;
    __HAL_TIM_SET_COMPARE(htim_delay, tim_channel[i], 0);
  }

  HAL_TIM_RegisterCallback(htim_delay, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, delay_timer_callback);
  __HAL_TIM_SET_COUNTER(htim_delay, 0xFFFFFFFE);
  HAL_TIM_Base_Start(htim_delay);

  return;
}

void delay_timer_callback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (callback[0] != NULL)
		{
			callback[0]();
			callback[0] = NULL;
		}
		else
		{
			// Error: Timer interrupt with no callback assigned
		}
	}
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (callback[1] != NULL)
		{
			callback[1]();
			callback[1] = NULL;
		}
		else
		{
			// Error: Timer interrupt with no callback assigned
		}
	}
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if (callback[2] != NULL)
		{
			callback[2]();
			callback[2] = NULL;
		}
		else
		{
			// Error: Timer interrupt with no callback assigned
		}
	}
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if (callback[3] != NULL)
		{
			callback[3]();
			callback[3] = NULL;
		}
		else
		{
			// Error: Timer interrupt with no callback assigned
		}
	}
}

/* End of file -------------------------------------------------------- */
