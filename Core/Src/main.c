// PLAN
//--> 2 timerja
//    --> timer 3
//        --> interrupt vsakih 50ms
//        --> stej do 80 (4s) prenesi polji Ay in Ax (DMA)
//        --> stej do 400 (20s) --> analiza podatkov
//    --> timer 4 pwm za flashanje ledic
//--> polji Ay in Ax (80 meritev, ena meritev vsakih 50ms),
//    polje B (800 meritev, DMA prenos iz Ay, Ax vsake 4s). Ko se napolni, izvedemo analizo podatkov
//--> analiza podatkov
//    ce je odstopanje v podatkih visoko, zacnemo naslednih 20s flashat lucke.

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

/* USER CODE BEGIN PFP */

TIM_HandleTypeDef timer;
TIM_HandleTypeDef timer2;

uint8_t Ax[80];
uint8_t Ay[80];
uint8_t B[900];

uint8_t in;
uint8_t in2;
uint8_t out;


SPI_HandleTypeDef hspi1;

DMA_HandleTypeDef dma;
DMA_InitTypeDef  DMA_InitStruct;




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


extern  int cnt1;
extern int cnt2;

void func(){


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			 out = 0x29 | 0x80; // 0xa9
			 HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
			 HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

			 Ax[cnt1]=abs(in);



			 // beri naklon po y osi -> Naslov 0x2B
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			 out = 0x2B | 0x80; // 0xaB
			 HAL_SPI_TransmitReceive(&hspi1, &out, &in2, 1, HAL_MAX_DELAY);
			 HAL_SPI_TransmitReceive(&hspi1, &out, &in2, 1, HAL_MAX_DELAY);
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

			 Ay[cnt1]=abs(in2);





			 cnt1++;

			 if(cnt1>=80){

				 //vsake 4 sekunde DMA prenos




				 HAL_DMA_PollForTransfer(&dma, HAL_DMA_FULL_TRANSFER,
				 	HAL_MAX_DELAY);
				 HAL_DMA_Start(&dma, Ax, &B[cnt2 * 80],
				 			80 * sizeof(uint8_t));




				 //HAL_DMA_PollForTransfer(&dma, HAL_DMA_FULL_TRANSFER,
					//		 	HAL_MAX_DELAY);




				 HAL_DMA_Start(&dma, Ay,


				 			&B[400 + (80 * cnt2)],80*sizeof(uint8_t));
				 //HAL_DMA_Start(&dma,(uint32_t)&Ay,(uint32_t)&B,80*8);




				 cnt1=0;
				 cnt2++;






			 }
			 if(cnt2>=5)

			 {
				 //izracnaj povprecje

				 cnt2=0;


				 double average, variance, std_deviation, sum_two = 0;

				 	uint32_t sum = 0;
				 	// compute avg
				 	for (int32_t i = 0; i < 800; i++) {
				 		sum += (uint32_t) B[i];
				 	}

				 	average = sum / 400.0;

				 	// Compute  variance  and standard deviation
				 	for (int32_t i = 0; i < 800; i++) {
				 		sum_two += pow((((uint32_t)B[i]) - average), 2);
				 	}
				 	variance = sum_two / 400.0;
				 	std_deviation = sqrt(variance);




				 	if (std_deviation > 20 ){


				 		__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, 50 );
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, 50 );
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, 50 );
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_4, 50 );
				 	}else{


				 		__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, 0);
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, 0);
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, 0);
				 			__HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_4, 0);
				 	}








			 }








}




/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */



		     __DMA2_CLK_ENABLE();

		     DMA_InitStruct.Channel=DMA_CHANNEL_0; //mem to mem periph je source pri m2m

		     DMA_InitStruct.Direction  = DMA_MEMORY_TO_MEMORY;

		     DMA_InitStruct.PeriphInc = DMA_PINC_ENABLE;

		     DMA_InitStruct.MemInc = DMA_MINC_ENABLE; //increment

		     DMA_InitStruct.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		     DMA_InitStruct.MemDataAlignment = DMA_MDATAALIGN_BYTE;

		     DMA_InitStruct.Mode = DMA_NORMAL;//v

		     DMA_InitStruct.Priority = DMA_PRIORITY_MEDIUM;

		     DMA_InitStruct.FIFOMode = DMA_FIFOMODE_ENABLE;
		     DMA_InitStruct.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		     DMA_InitStruct.MemBurst = DMA_MBURST_SINGLE;
		     DMA_InitStruct.PeriphBurst = DMA_PBURST_SINGLE;

		     dma.Instance = DMA2_Stream0;
		     dma.Init=DMA_InitStruct;


		     HAL_DMA_Init(&dma);




	__HAL_RCC_SPI1_CLK_ENABLE();

	      hspi1.Instance = SPI1;
	      hspi1.Init.Mode = SPI_MODE_MASTER;
	      hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	      hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	      hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	      hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	      hspi1.Init.NSS = SPI_NSS_SOFT;
	      hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	      hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	      hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	      HAL_SPI_Init(&hspi1);

	      __HAL_RCC_GPIOA_CLK_ENABLE();

	        GPIO_InitTypeDef init_structure;
	        init_structure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	        init_structure.Pull = GPIO_NOPULL;
	        init_structure.Speed = GPIO_SPEED_FREQ_LOW;
	        init_structure.Mode = GPIO_MODE_AF_PP;
	        init_structure.Alternate = GPIO_AF5_SPI1;
	        HAL_GPIO_Init(GPIOA, &init_structure);


	    __HAL_RCC_GPIOE_CLK_ENABLE();
	  	init_structure.Pin = GPIO_PIN_3;
	  	init_structure.Pull = GPIO_NOPULL;
	  	init_structure.Speed = GPIO_SPEED_FREQ_LOW;
	  	init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	  	HAL_GPIO_Init(GPIOE, &init_structure);
	  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	  	 // slave deselect
	  		           HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	  		           out = 0x8F;
	  		           HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
	  		           HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
	  		           HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	  		           HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	  		           out = 0x20;
	  		           HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
	  		           out = 0x47;
	  		           HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, HAL_MAX_DELAY);
	  		           HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	  		         HAL_Delay(500);


	  		       __HAL_RCC_GPIOD_CLK_ENABLE();
	  		         // init leds
	  		         GPIO_InitTypeDef leds;
	  		         leds.Pin =  GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	  		         leds.Mode = GPIO_MODE_AF_PP;
	  		         leds.Alternate = GPIO_AF2_TIM4;
	  		         leds.Pull = GPIO_NOPULL;
	  		         leds.Speed = GPIO_SPEED_FREQ_LOW;

	  		         HAL_GPIO_Init(GPIOD,&leds);

	  		       __HAL_RCC_TIM4_CLK_ENABLE();


	  		        timer.Instance = TIM4;
	  		        timer.Init.CounterMode = TIM_COUNTERMODE_UP;
	  		        timer.Init.Period = 50 - 1; // 5 ms P = (ARR+1)*(1/(Clk_Freq/(Prescaler+1))
	  		        timer.Init.Prescaler = 1600 - 1; // 1/(16MhZ / 1600) = 0.1ms
	  		        HAL_TIM_PWM_Init(&timer);

	  		        TIM_OC_InitTypeDef PWM_channel;
	  		        PWM_channel.OCMode = TIM_OCMODE_PWM1;
	  		        PWM_channel.OCPolarity = TIM_OCPOLARITY_HIGH;

	  		        PWM_channel.Pulse = 50 - 1;
	  		        HAL_TIM_PWM_ConfigChannel(&timer, &PWM_channel, TIM_CHANNEL_1);
	  		        HAL_TIM_PWM_ConfigChannel(&timer, &PWM_channel, TIM_CHANNEL_3);

	  		        PWM_channel.Pulse = 0;
	  		        HAL_TIM_PWM_ConfigChannel(&timer, &PWM_channel, TIM_CHANNEL_2);
	  		        HAL_TIM_PWM_ConfigChannel(&timer, &PWM_channel, TIM_CHANNEL_4);

	  		        HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
	  		        HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);
	  		        HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_3);
	  		        HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_4);





	  		      __HAL_RCC_TIM3_CLK_ENABLE();


	  		        timer2.Instance = TIM3;
	  		        timer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  		        timer2.Init.Period = 50 - 1; // 50 ms P = (ARR+1)*(1/(Clk_Freq/(Prescaler+1))
	  		        timer2.Init.Prescaler = 16000 - 1; // 1/(16MhZ / 16000) = 1 ms
	  		        HAL_TIM_Base_Init(&timer2);

	  		        __HAL_TIM_ENABLE_IT(&timer2,TIM_IT_UPDATE);

	  		        HAL_TIM_Base_Start(&timer2);

	  		        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 2);
	  		        HAL_NVIC_EnableIRQ(TIM3_IRQn);







	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}



/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : I2S3_WS_Pin */
	GPIO_InitStruct.Pin = I2S3_WS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
	GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
	GPIO_InitStruct.Pin = Audio_SCL_Pin | Audio_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
