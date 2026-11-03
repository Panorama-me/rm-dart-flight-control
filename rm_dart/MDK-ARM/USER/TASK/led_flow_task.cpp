#include "led_flow_task.h"
#include "Tasks.h"


#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//À¶ -> ÂÌ(Ãð) -> ºì -> À¶(Ãð) -> ÂÌ -> ºì(Ãð) -> À¶ 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

/**
* @brief Function implementing the led_flow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_flow_task */
float num=1;
void led_flow_task(void const * argument)
{
	  uint16_t i, j;
    fp32 delta_alpha, delta_red, delta_green, delta_blue;
    fp32 alpha,red,green,blue;
    uint32_t aRGB;
		buzzer_warn_error(num);

  /* USER CODE BEGIN led_flow_task */
  /* Infinite loop */
  for(;;)
  {
     for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
      {
            alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
            red   = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
            green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
            blue  = ((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
            delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
            delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
            delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                aRGB_led_show(aRGB);
                osDelay(1);
            }
		 }
  }
}
/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          Ê¹µÃ·äÃùÆ÷Ïì
  * @param[in]      num:ÏìÉù´ÎÊý
  * @retval         none
  */
 void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}
