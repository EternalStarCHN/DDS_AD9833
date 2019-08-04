/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "gpio.h"

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	int control;
	
	void delay_us(unsigned int m)
	{
		unsigned int  n;
		for (n=0;n<=m;n++)
		{
		}
	}
	
	static void AD9833_Delay(void)
	{
		uint16_t i;
		for (i = 0; i < 1; i++);
	}
	
		void AD9833_Write(unsigned int TxData)
	{
		unsigned char i;

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk = 1
		//AD9833_Delay();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //en = 1
		//AD9833_Delay();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //en = 0
		//AD9833_Delay();
		for(i = 0; i < 16; i++)
		{
						if (TxData & 0x8000)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //dat = 1
						else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); //dat = 0
						
						AD9833_Delay();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //clk = 0
						AD9833_Delay();                
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk = 1
						
						TxData <<= 1;
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //en = 1
		
	} 

	void AD9833_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase )
	{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;


		frequence_mid=268435456/25;
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1000000;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA; 
		frequence_LSB=frequence_hex; 
		frequence_LSB=frequence_LSB&0x3fff;
		frequence_MSB=frequence_hex>>14; 
		frequence_MSB=frequence_MSB&0x3fff;

		Phs_data=Phase|0xC000;     
		AD9833_Write(0x0100); 
		AD9833_Write(0x2100); 

		if(Freq_SFR==0)                               
		{
						 frequence_LSB=frequence_LSB|0x4000;
						 frequence_MSB=frequence_MSB|0x4000;
				
						AD9833_Write(frequence_LSB); 
						AD9833_Write(frequence_MSB);
						AD9833_Write(Phs_data);        
		}
		if(Freq_SFR==1)                             
		{
						 frequence_LSB=frequence_LSB|0x8000;
						 frequence_MSB=frequence_MSB|0x8000;

						AD9833_Write(frequence_LSB); 
						AD9833_Write(frequence_MSB); 
						AD9833_Write(Phs_data);        
						
		}

		if(WaveMode==1)
						 AD9833_Write(0x2002); 
		if(WaveMode==2)       
						AD9833_Write(0x2028); 
		if(WaveMode==0)       
						AD9833_Write(0x2000); 

	}
	void write_2byte(uint16_t a)
	{
		unsigned char i ;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk = 1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //dat = 1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //en = 1
		control = 0;
		delay_us(1000);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk = 1
		delay_us(2);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //en = 0
		for( int i=0 ; i<16 ; i++ )
		{
			if(a&0x8000)
			{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //dat	= 1
			}
			else
			{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); //dat	= 0
			}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //clk = 0
		delay_us(5);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk =1
			a=a<<1;
		}
		delay_us(2);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //en = 1 
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //clk = 0
		delay_us(1000);
		control = 1;
	}
	
	void init_dds(void){
		write_2byte(0x2100);
		write_2byte(0x2000);
		write_2byte(0x4000);
		write_2byte(0x403F);
		write_2byte(0x2900);
		write_2byte(0x8000);
		write_2byte(0x803F);
		write_2byte(0xC000);
		write_2byte(0xF000);
		write_2byte(0x2000);
	}
	
	void output(unsigned long freq_value , int mode){
		unsigned long dds;
		uint16_t dds1,dds2;
		dds = freq_value * 10.7374185;//18.64135111111;
		dds = dds << 2;
		dds1 = dds;
		dds2 = dds >> 16;
		dds1 = dds1 >> 2;
		dds2 = dds2 & 0x7FFF;
		dds2 = dds2 | 0x4000;
		dds1= dds1 & 0x7FFF;
		dds1= dds1 | 0x4000;
		if(mode == 0)
		{
			write_2byte(0x2000);
		}
		else if(mode == 1)
		{
			write_2byte(0x2002);
		}
		else if(mode == 2)
		{
			write_2byte(0x2028);
		}
		write_2byte(dds1);
		write_2byte(dds2);
	}
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	init_dds();
//	output(10000,0); 
//		AD9833_WaveSeting(1000000,0,0,0);
  while (1)
  {
    /* USER CODE END WHILE */
	for(int i=1; i<101 ;i++){
		AD9833_WaveSeting(i*10000,0,0,0);
		HAL_Delay(100);
	}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
