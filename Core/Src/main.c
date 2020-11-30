/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t mode=0;
#define multiplex_display 0x1
#define shift_display 0x2
#define up_or_down	0x04

#define max_buff_size 64
uint8_t active_display=0;
uint16_t display_buffer_pa[max_buff_size];
uint16_t display_buffer_pb[max_buff_size];
uint8_t str[max_buff_size];

uint16_t size_buff=0;
uint16_t offset=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void my_str_cpy(uint8_t * from, uint8_t * to, uint16_t *copied, uint16_t max){
	uint16_t cnt=0;
	for(cnt=0;cnt<max;cnt++){
		*to=*from;
		if(*from == '\0')
			{*copied=cnt;return;}
		else
			{to+=1;from+=1;}
	}
}


void convert_str_to_7seg(uint8_t *from, uint16_t *pa,uint16_t *pb,uint16_t max){
	uint16_t cnt=0;//,mask_a=0,mask_b=0;
	for (cnt=0;cnt<max;cnt++){
		*pa=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
		*pb=0xffff;
		switch(*from){
			case '0':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin);
				break;}
			case '1':{
				*pa&=~(seg_B_pa_Pin | seg_C_pa_Pin);
				break;}
			case '2':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '3':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '4':{
				*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin );
				break;}
			case '5':{
				*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '6':{
				*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '7':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin );
				break;}
			case '8':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '9':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}

			case 'A':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'a':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'b':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'C':{
				*pa &= ~(seg_A_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'c':{
				*pa &= ~( seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'd':{
				*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'E':{
				*pa &= ~(seg_A_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'F':{
				*pa &= ~(seg_A_pa_Pin |  seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'G':{
				*pa &= ~(seg_A_pa_Pin |  seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'H':{
				*pa &= ~(seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'h':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'I':{
				*pa &= ~( seg_E_pa_Pin | seg_F_pa_Pin);
				// *pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'J':{
				*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'L':{
				*pa &= ~(seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'n':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'O':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'o':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'P':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin  | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'q':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
				*pb &= ~( seg_G_pb_Pin);
				break;}
			case 'r':{
				*pa &= ~( seg_E_pa_Pin );
				*pb &= ~(seg_G_pb_Pin);
				break;}
			case 'S':{
				*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin  | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 't':{
				*pa &= ~( seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'U':{
				*pa &= ~(  seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'u':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'y':{
				*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin  | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}

			//specials:
			case 'K':{
				*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_G_pb_Pin);
				break;}
			case 'M':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'V':{
				*pa &= ~( seg_B_pa_Pin |   seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			case 'W':{
				*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'X':{
				*pa &= ~(seg_A_pa_Pin  );
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case 'Z':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
				break;}
			case '_':{
				//*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
				*pb &= ~(seg_D_pb_Pin );
				break;}
			default:
				*pb &=~seg_DP_pb_Pin;
			// specials:
			/*case 'm':{
				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~( seg_G_pb_Pin | seg_DP_pb_Pin);

				pa+=1;pb+=1;
				*pa=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
				*pb=0xffff;

				*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
				*pb &= ~( seg_G_pb_Pin );
				break;}
			case 'x':{
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
				*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin | seg_DP_pb_Pin);

				pa+=1;pb+=1;
				*pa=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
				*pb=0xffff;
				*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
								*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);

			}*/


		}
		pa+=1;pb+=1;from+=1;
	}
}

void start_tim17_with_IT(void){
	  LL_TIM_EnableIT_UPDATE(TIM17);
	  TIM17->CR1 |= TIM_CR1_CEN; // start timer
}

void start_tim16_with_IT(void){
	LL_TIM_EnableIT_UPDATE(TIM16);
	TIM16->CR1 |= TIM_CR1_CEN; // start timer
}

void multiplex_display_fcn(uint16_t offset,uint16_t max_offset){
	//LL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
	active_display++;
	if (active_display>=4)
		active_display=0;
	if (offset>max_offset){
		offset=max_offset;}
	switch (active_display){
		case 0:{
			GPIOA->ODR = display_buffer_pa[offset];
			GPIOB->ODR = display_buffer_pb[offset];
			GPIOA->ODR |= dig_0_pa_Pin;
			break;}
		case 1:{
			GPIOA->ODR = display_buffer_pa[offset+1];
			GPIOB->ODR = display_buffer_pb[offset+1];
			GPIOA->ODR |= dig_1_pa_Pin;
			break;}
		case 2:{
			GPIOA->ODR = display_buffer_pa[offset+2];
			GPIOB->ODR = display_buffer_pb[offset+2];
			GPIOA->ODR |= dig_2_pa_Pin;
			break;}
		case 3:{
			GPIOA->ODR = display_buffer_pa[offset+3];
			GPIOB->ODR = display_buffer_pb[offset+3];
			GPIOA->ODR |= dig_3_pa_Pin;
			break;}
	}


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint16_t temp=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


  my_str_cpy((uint8_t *) "PEtEr_SZabo_47447_PIrHaLa_MatEJ_92621", str, &size_buff, max_buff_size);
  convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
  start_tim17_with_IT();
  start_tim16_with_IT();
  //GPIOA->ODR=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
  //GPIOB->ODR=0xffff;

  //GPIOA->ODR &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
  //GPIOB->ODR &= ~( 1<<5);

  //GPIOA->ODR |= dig_1_pa_Pin;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __WFI();
	  if (mode & multiplex_display){
		  multiplex_display_fcn(offset,size_buff-4);
		  mode &=~ multiplex_display;
	  }
	  if (mode & shift_display){
		  if (mode & up_or_down){
			  // up
			  offset++;
			  if (offset >(size_buff-4)){
				  offset=size_buff-4;
				  mode &= ~up_or_down;
			  }
		  }
		  else{
			  if (offset == 0){
				  offset=0;
				  mode |=up_or_down;
			  }
			  else{
				  offset--;
			  }
		  }


		  //offset++;
		  //LL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
		  //if (offset >(size_buff-4)){
		  	  //offset=0;}
		  mode &=~ shift_display;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 7999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, seg_B_pa_Pin|seg_A_pa_Pin|dig_3_pa_Pin|seg_F_pa_Pin
                          |dig_1_pa_Pin|dig_0_pa_Pin|dig_2_pa_Pin|dig_time_pa_Pin
                          |seg_C_pa_Pin|seg_E_pa_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, seg_DP_pb_Pin|led_Pin|seg_G_pb_Pin|seg_D_pb_Pin);

  /**/
  GPIO_InitStruct.Pin = seg_B_pa_Pin|seg_A_pa_Pin|dig_3_pa_Pin|seg_F_pa_Pin
                          |dig_1_pa_Pin|dig_0_pa_Pin|dig_2_pa_Pin|dig_time_pa_Pin
                          |seg_C_pa_Pin|seg_E_pa_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = seg_DP_pb_Pin|led_Pin|seg_G_pb_Pin|seg_D_pb_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
