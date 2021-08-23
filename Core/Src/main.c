/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
struct Cal
{
	int32_t mean;
	int32_t diff;
};

int32_t JS[2], Cur_ENC1, Prev_ENC1, Delta_ENC1, Cur_ENC2, Prev_ENC2, Delta_ENC2;
uint32_t Spd, Nstep;
uint8_t i;
extern uint8_t* buffer[64];
extern uint32_t len;
char *ok = "OK\n";
char *done = "DONE\n";
struct Cal JS_Cal_X, JS_Cal_Y;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
 
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
 
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
 
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
 
    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

void ADC_Select_CH0 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

char* itoa(int value, char* result, int base)
{
	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;
	do
	{
		tmp_value = value;
		value = value / base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while(value);
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr)
	{
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
	
}

int32_t ADC_Read_CH0(void)
{
	int32_t res;	
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	res = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);	
	return res; 
}

int32_t ADC_Read_CH1(void)
{
	int32_t res;	
	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	res = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);	
	return res; 
}

float min(float x, float y)
{
    if (x <= y)
		{
        return x;
		}
    else
		{
        return y;
		}
}

float max(float x, float y)
{
    if (x >= y)
		{
        return x;
		}
    else
		{
        return y;
		}
}
uint32_t to_num(uint8_t* num)
{
	if (num == "0")
	{
		return 0;
	}
	if (num == "1")
	{
		return 1;
	}
	if (num == "2")
	{
		return 2;
	}
	if (num == "3")
	{
		return 3;
	}
	if (num == "4")
	{
		return 4;
	}
	if (num == "5")
	{
		return 5;
	}
	if (num == "6")
	{
		return 6;
	}
		if (num == "7")
	{
		return 7;
	}
		if (num == "8")
	{
		return 8;
	}
		if (num == "9")
	{
		return 9;
	}
	return 0;
}
struct Cal JS_Calibration(int CH)
{
	struct Cal res;
	int mini = 4097, maxi = 0, rn, i;	
	res.mean = 0;
	if (CH == 0)
	{
		ADC_Select_CH0();
		for (i=0; i<1000; i++)
		{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		rn = HAL_ADC_GetValue(&hadc1);
		res.mean += rn;
		mini = min(rn, mini);
		maxi = max(rn ,maxi);
		}
		res.mean /= 1000;
		HAL_ADC_Stop(&hadc1);	
		res.diff = max(res.mean - mini, maxi - res.mean) + 16;

	}
	else
	{
		ADC_Select_CH1();
		for (i=0; i<1000; i++)
		{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		rn = HAL_ADC_GetValue(&hadc1);
		res.mean += rn;
		mini = min(rn, mini);
		maxi = max(rn ,maxi);
		}
		res.mean /= 1000;
		HAL_ADC_Stop(&hadc1);	
		res.diff = max(res.mean - mini, maxi - res.mean) + 16;

	}
	return res; 
}
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	len = 0;
	memset(buffer, '\0', 64); 
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
	MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	Prev_ENC1 = 32767 - ((__HAL_TIM_GET_COUNTER(&htim2) - 1) & 0xFFFF);
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	Prev_ENC2 = 32767 - ((__HAL_TIM_GET_COUNTER(&htim3) - 1) & 0xFFFF);
	
	DWT_Delay_Init();
		
	JS_Cal_X = JS_Calibration(0);
	JS_Cal_Y = JS_Calibration(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t* alphabet[7];
	alphabet[0] = (uint8_t*) "";
	if (HAL_GPIO_ReadPin(PC_GPIO_Port, PC_Pin))
	{
		while (1)
		{
		HAL_Delay(1000);
		if (buffer[0] == alphabet[0])
		{
		CDC_Transmit_FS((uint8_t * ) done, strlen(done));
		HAL_Delay(500);
		}
		CDC_Transmit_FS((uint8_t * ) buffer[0], 1);
		HAL_Delay(500);
		CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
		HAL_Delay(500);
		if (buffer[0] != alphabet[0])
		{
		CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
		}
		if (buffer[0] == alphabet[0])
		{
    CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
    if (buffer[1] == (uint8_t * )
      "E") {
      CDC_Transmit_FS(buffer[1], 1);
      if (buffer[2] == (uint8_t * )
        "+") {
        CDC_Transmit_FS(buffer[2], 1);
        if (buffer[3] == (uint8_t * )
          "D") {
          CDC_Transmit_FS(buffer[3], 1);
          CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
        }
      }
      if (buffer[2] == (uint8_t * )
        "-") {
        CDC_Transmit_FS(buffer[2], 1);
        if (buffer[3] == (uint8_t * )
          "D") {
          CDC_Transmit_FS(buffer[3], 1);
          CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
        }
      }
    }
    if (buffer[1] == (uint8_t * )
      "F") {
      CDC_Transmit_FS(buffer[1], 1);
      if (buffer[2] == (uint8_t * )
        "+") {
        CDC_Transmit_FS(buffer[2], 1);
        if (buffer[9] == (uint8_t * )
          "D") {
          CDC_Transmit_FS(buffer[3], 1);
          CDC_Transmit_FS(buffer[4], 1);
          CDC_Transmit_FS(buffer[5], 1);
          CDC_Transmit_FS(buffer[6], 1);
          CDC_Transmit_FS(buffer[7], 1);
          CDC_Transmit_FS(buffer[8], 1);
          CDC_Transmit_FS(buffer[9], 1);
          CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
          Nstep = (uint32_t)(100000 * to_num(buffer[3]) +
            10000 * to_num(buffer[4]) + 1000 * to_num(buffer[5]) +
            100 * to_num(buffer[6]) + 10 * to_num(buffer[7]) + to_num(buffer[8]));
        }
      }
      if (buffer[2] == (uint8_t * )
        "-") {
        CDC_Transmit_FS(buffer[2], 1);
        if (buffer[9] == (uint8_t * )
          "D") {
          CDC_Transmit_FS(buffer[3], 1);
          CDC_Transmit_FS(buffer[4], 1);
          CDC_Transmit_FS(buffer[5], 1);
          CDC_Transmit_FS(buffer[6], 1);
          CDC_Transmit_FS(buffer[7], 1);
          CDC_Transmit_FS(buffer[8], 1);
          CDC_Transmit_FS(buffer[9], 1);
          CDC_Transmit_FS((uint8_t * ) ok, strlen(ok));
          Nstep = (uint32_t)(100000 * to_num(buffer[3]) +
            10000 * to_num(buffer[4]) + 1000 * to_num(buffer[5]) +
            100 * to_num(buffer[6]) + 10 * to_num(buffer[7]) + to_num(buffer[8]));
        }
      }
    }
  }

}
	

		
	}
	else
	{
  while (1)
  {
		
		//XY
		JS[0] = ADC_Read_CH0() - JS_Cal_X.mean;
		JS[1] = ADC_Read_CH1() - JS_Cal_Y.mean;
		if (abs(JS[0]) > JS_Cal_X.diff)
		{
			if (JS[0] > 0) 
			{
				HAL_GPIO_WritePin(GPIOB, DIR1_Pin, GPIO_PIN_SET);
				Spd = (uint32_t) (1300 - 1280 * JS[0] / 2048);
				HAL_GPIO_WritePin(GPIOB, ST1_Pin, GPIO_PIN_SET);
				DWT_Delay_us(Spd);
				HAL_GPIO_WritePin(GPIOB, ST1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, DIR1_Pin, GPIO_PIN_RESET);
				Spd = (uint32_t) (1300 + 1280 * JS[0] / 2048);
				HAL_GPIO_WritePin(GPIOB, ST1_Pin, GPIO_PIN_SET);
				DWT_Delay_us(Spd);
				HAL_GPIO_WritePin(GPIOB, ST1_Pin, GPIO_PIN_RESET);
			}
		}
	  if (abs(JS[1]) > JS_Cal_Y.diff)
		{
			if (JS[1] > 0) 
			{
				HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_SET);
				Spd = (uint32_t) (2500 - 2300 * JS[1] / 2048);
				HAL_GPIO_WritePin(GPIOB, ST2_Pin, GPIO_PIN_SET);
				DWT_Delay_us(Spd);
				HAL_GPIO_WritePin(GPIOB, ST2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_RESET);
				Spd = (uint32_t) (2500 - 2300 * abs(JS[1]) / 2048);
				HAL_GPIO_WritePin(GPIOB, ST2_Pin, GPIO_PIN_SET);
				DWT_Delay_us(Spd);
				HAL_GPIO_WritePin(GPIOB, ST2_Pin, GPIO_PIN_RESET);
			}
		}
		
		//Z
		Cur_ENC1 = 32767 - ((__HAL_TIM_GET_COUNTER(&htim2) - 1) & 0xFFFF);
		if (Cur_ENC1 > 16384) 
		{
			Cur_ENC1 = Cur_ENC1 - 32768;
		}
		if (Cur_ENC1 != Prev_ENC1)
		{
			Delta_ENC1 = Cur_ENC1 - Prev_ENC1;
			if (Delta_ENC1 > 0)
			{
				HAL_GPIO_WritePin(GPIOB, DIR3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
			}
						if (Delta_ENC1 < 0)
			{
				HAL_GPIO_WritePin(GPIOB, DIR3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_SET);
				DWT_Delay_us(100);
				HAL_GPIO_WritePin(GPIOB, ST3_Pin, GPIO_PIN_RESET);
			}
		}
		Prev_ENC1 = Cur_ENC1;
		
		Cur_ENC2 = 32767 - ((__HAL_TIM_GET_COUNTER(&htim3) - 1) & 0xFFFF);
		
		//PHI
		if (HAL_GPIO_ReadPin(F_GPIO_Port, F_Pin))
		{
			if (Cur_ENC2 > 16384) 
			{
				Cur_ENC2 = Cur_ENC2 - 32768;
			}
			if (Cur_ENC2 != Prev_ENC2)
			{
				Delta_ENC2 = Cur_ENC2 - Prev_ENC2;
				if (Delta_ENC2 > 0)
				{
					HAL_GPIO_WritePin(GPIOA, DIR4_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
				}
							if (Delta_ENC2 < 0)
				{
					HAL_GPIO_WritePin(GPIOA, DIR4_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOA, ST4_Pin, GPIO_PIN_RESET);
				}
			}
		}
		
		//FOCUS
		
	  else 
		{
			if (Cur_ENC2 > 16384) 
			{
				Cur_ENC2 = Cur_ENC2 - 32768;
			}
			if (Cur_ENC2 != Prev_ENC2)
			{
				Delta_ENC2 = Cur_ENC2 - Prev_ENC2;
				if (Delta_ENC2 > 0)
				{
					HAL_GPIO_WritePin(GPIOA, DIR5_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
				}
							if (Delta_ENC2 < 0)
				{
					HAL_GPIO_WritePin(GPIOA, DIR5_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_SET);
					DWT_Delay_us(200);
					HAL_GPIO_WritePin(GPIOB, ST5_Pin, GPIO_PIN_RESET);
				}
			}
		}
		
		Prev_ENC2 = Cur_ENC2;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
