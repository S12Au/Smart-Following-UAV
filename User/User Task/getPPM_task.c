#include "FreeRTOS.h"
#include "queue.h"
#include "GY86.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "semphr.h"
#include "tim.h"
#include "i2c.h"

#define YES_FIRST	1
#define NO_FIRST 	0

static uint32_t g_last_capture = 0;
static uint32_t g_now_capture;
static uint16_t g_ppmCh[9];
static uint8_t g_ppmChi = 0;
static uint8_t g_first = YES_FIRST;
static uint8_t g_test = 0;
QueueHandle_t QueuePPM;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

struct PPM_Data{
	uint16_t ppmCh[9];
} idata;

void getPPM_Task(){
	// 确保定时器时钟已经使能
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	// 启动输入捕获中断
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);  // 启动通道2输入捕获中断
	
	// 添加调试信息
	printf("PPM Task Started!\r\n");
	
	// 任务主体可以为空或者添加其他逻辑
	for(;;) {
		vTaskDelay(1000); // 避免任务过快循环
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	xHigherPriorityTaskWoken = pdFALSE;
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		g_now_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		
		if (g_ppmChi == 8)
		{
			xQueueSendFromISR(QueuePPM, &idata, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			
		}
		
		if (g_now_capture - g_last_capture > 4000)
		{
			g_last_capture = g_now_capture;
			g_ppmChi = 0;
		}
		else
		{
			idata.ppmCh[g_ppmChi] = g_now_capture - g_last_capture;
			g_ppmChi++;
			g_last_capture = g_now_capture;
		}
	}
}