/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "tim.h"
#include "stdio.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "bsp_imu.h"
float speed = 0.2;
char buf[300];
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;
uint32_t ADC_Value[100];
uint32_t ad1,ad2,ad3,ad4;
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void set_total_angle(moto_measure_t *p, moto_measure_t *p1, int angle, u8 dir, u16 speed);
void weizhihuan(moto_measure_t *p, int angle);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of myTask02 */
    osThreadDef(myTask02, StartTask02, osPriorityBelowNormal, 0, 128);
    myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

    /* definition and creation of myTask03 */
    osThreadDef(myTask03, StartTask03, osPriorityBelowNormal, 0, 128);
    myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    my_can_filter_init_recv_all(&hcan1);
    my_can_filter_init_recv_all(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );	// Enable interrupts
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING );	// Enable interrupts
    //	PID_struct_init(&pid_omg, POSITION_PID, 20000, 20000,
    //									1.5f,	0.1f,	0.0f	);  //angular rate closeloop.
    for(int i = 0; i < 4; i++)
    {
        PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
                        8.7f,	0.001f, 0.0f	); //4 motos angular rate closeloop.. 1.5f,	0.001f, 0.0f  0.09f,	0.0f, 0.45f weizhi_pid

        PID_struct_init(&weizhi_pid[i], POSITION_PID, 20000, 20000,
                        0.72f,	0.00f, 0.3f	);
        PID_struct_init(&shudu_pid[i], POSITION_PID, 20000, 20000,
                        1.5f,	0.0f, 0.0f	);
    }
		
		  for(int i = 4; i < 8; i++)//堕轮转向Pid
    {
        PID_struct_init(&weizhi_pid[i], DELTA_PID, 20000, 20000,
                        0.72f,	0.00f, 0.3f	);
        PID_struct_init(&shudu_pid[i], POSITION_PID, 20000, 20000,
                        1.5f,	0.0f, 0.0f	);
    }
	 PID_struct_init(&yaw_pos,DELTA_PID,20000,20000,10.0f,0.0f,0.0f);//yaw pid
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 1000);	//ppm must be 1000 at first time.
    HAL_Delay(100);
    /* Infinite loop */

    /* Infinite loop */
    for(;;)
    {
//		 set_total_angle(&moto_chassis[1], &moto_chassis[0], 8192 * 19.2 * 5,  1, 500 );
//		  while(1)
//        {
//		 pid_calc(&pid_spd[0],moto_chassis[0].speed_rpm,0);
//            pid_calc(&pid_spd[1], moto_chassis[1].speed_rpm, 0);
//            set_moto_current(&hcan1, pid_spd[0].pos_out,
//                             pid_spd[1].pos_out,
//                             pid_spd[2].pos_out,
//                             pid_spd[3].pos_out);
        weizhihuan(&moto_chassis[1], 8192 * 19.2 * 10);//舵轮速度
			   weizhihuan(&moto_chassis[5], 8192 * 19.2 * 2);//舵轮转向
        osDelay(5);
//        }

        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)//读取角度
{
    /* USER CODE BEGIN StartTask02 */
	  u8 i;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 100);
    /* Infinite loop */
    for(;;)
    {
        for(i = 0, ad1 = 0, ad2 = 0; i < 100;) {
            ad1 += ADC_Value[i++];
            ad2 += ADC_Value[i++];
					  ad3 += ADC_Value[i++];
					  ad4 += ADC_Value[i++];
        }
        ad1 /= 25;
        ad2 /= 25;
				ad3 /= 25;
        ad4 /= 25;
				printf("AD1_value=%1.3fV  AD2_value=%1.3fV  AD3_value=%1.3fV  AD4_value=%1.3fV\r\n", ad1*3.3f/4096,ad2*3.3f/4096,ad3*3.3f/4096,ad4*3.3f/4096);
				ad1=ad2=ad3=ad4=0;
        osDelay(1);
    }
    /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)//读取imu
{
    /* USER CODE BEGIN StartTask03 */
    /* Infinite loop */
    for(;;)
    {
        mpu_get_data();
        imu_ahrs_update();
        imu_attitude_update();
        sprintf(buf, " Roll: %8.3lf    Pitch: %8.3lf    Yaw: %8.3lf\r\n", imu.rol, imu.pit, imu.yaw);
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf) - 1), 55);
        HAL_Delay(5);
        osDelay(1);
    }
    /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void set_total_angle(moto_measure_t *p, moto_measure_t *p1, int angle, u8 dir, u16 speed)
{
//	 int round=0;
    get_total_angle(p);

//	 if(p->angle>p->last_angle)
//		 round=1;
    if(dir == 1) //+
    {
        while(p->total_angle < angle)
        {
//             printf("currend:%d  speed:%d \r\n",p->total_angle,p->speed_rpm);
            pid_calc(&pid_spd[0], p->speed_rpm, speed);
            printf("currend:%d  speed:%f \r\n", p->total_angle, pid_spd[0].pos_out);
//		  pid_calc(&pid_spd[1], p1->speed_rpm, -500);
            get_total_angle(p);
            set_moto_current(&hcan1, 0,
                             pid_spd[0].pos_out,
                             pid_spd[2].pos_out,
                             pid_spd[3].pos_out);
        }
    }
    else//-
    {   while(p->total_angle > -angle)
        {
//			pid_calc(&pid_spd[1], p1->speed_rpm, 500);
            pid_calc(&pid_spd[0], p->speed_rpm, -speed);
            printf("currend:%d\r\n", p->total_angle);
            get_total_angle(p);
//				abs_limit(&pid_spd[0].pos_out,5);
//			printf("total:%d\r\n",p->total_angle);
            set_moto_current(&hcan1, pid_spd[0].pos_out,
                             pid_spd[0].pos_out,
                             pid_spd[2].pos_out,
                             pid_spd[3].pos_out);
        }
    }


    pid_calc(&pid_spd[0], p->speed_rpm, 0);
    pid_calc(&pid_spd[1], p1->speed_rpm, 0);

}
void weizhihuan(moto_measure_t *p, int angle)
{

    pid_calc(&weizhi_pid[0], p->total_angle, angle); //位置环
//		get_total_angle(p);
    pid_calc(&pid_spd[0], p->speed_rpm, weizhi_pid[0].pos_out * speed);//速度环
//	  abs_limit(&pid_spd[0].pos_out, 2000);
//	  abs_limit(&pid_spd[0].pos_out, 1000);
    set_moto_current(&hcan1, 0,
                     pid_spd[0].pos_out,
                     0,
                     0);
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
