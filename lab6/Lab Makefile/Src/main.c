/**
  *
  * Seoin Kim
  * u1324614
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f072xb.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* Setup Functions */

/* USER CODE END PFP */
void SetEnable()
{
  // Enable GPIOB and GPIOC in the RCC.
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // Enable I2C2 in the RCC.
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
}

void InitializeLEDPins()
{
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr);
}

void Calibration()
{
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN by setting ADDIS*/
  /* (3) Clear DMAEN */
  /* (4) Launch the calibration by setting ADCAL */
  /* (5) Wait until ADCAL=0 */
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */ {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0);
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
  ADC1->CR |= ADC_CR_ADCAL; /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0); /* (5) */
}

void EnableADC()
{
  /* Enable the ADC */
  /* (1) Ensure that ADRDY = 0 */
  /* (2) Clear ADRDY */
  /* (3) Enable the ADC */
  /* (4) Wait until ADC ready */
  if (ADC1->ISR & ADC_ISR_ADRDY) /* (1) */ {
    ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
  }
  ADC1->CR |= ADC_CR_ADEN; /* (3) */
  while (!(ADC1->ISR & ADC_ISR_ADRDY)); /* (4) */    
}

void SetLEDSByADC()
{
  int threshold1 = 0;
  int threshold2 = 65;
  int threshold3 = 130;
  int threshold4 = 250;

  // Reset all LEDs
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  if (ADC1->DR > threshold1 && ADC1->DR <= threshold2) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  }
  else if (ADC1->DR > threshold2 && ADC1->DR <= threshold3) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  }
  else if (ADC1->DR > threshold3 && ADC1->DR <= threshold4) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  }
  else if (ADC1->DR > threshold4) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }
}
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) 
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  /****************************************** 
  6.1 Measuring a Potentiometer With the ADC 
  *******************************************/
  // 1. Initialize the LED pins to output.
  SetEnable();
  InitializeLEDPins();

  // 2. Select a GPIO pin to use as the ADC input. (PC0)
  // Configure to Analog mode
  GPIOC->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0);
  // No pull-up, pull-down
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_0);

  // 4. Configure the ADC.
  // 8-bit resolution
  ADC1->CFGR1 |= ADC_CFGR1_RES_1;
  ADC1->CFGR1 &= ~ADC_CFGR1_RES_0;
  // Continuous conversion mode
  ADC1->CFGR1 |= ADC_CFGR1_CONT;
  // Hardware triggers disabled
  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;

  // 5. Select/enable the input pin’s channel for ADC conversion.
  ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

  
  // 6. Perform a self-calibration, enable, and start the ADC.
    Calibration();
    EnableADC();
    // Start ADC.
    ADC1->CR |= ADC_CR_ADSTART;

  /************************************
  6.2 Generating Waveforms with the DAC
  *************************************/
  // 1. Select a GPIO pin to use as the DAC output (PA4)
  // Configure to Analog mode
  GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER4_0);
  // No pull-up, pull-down
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR4_0);
  
  // 2. Set the used DAC channel to software trigger mode
  DAC1->CR |= (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0);

  // 3. Enable the used DAC channel
  DAC1->CR |= DAC_CR_EN1;

  // 4. Copy one of the wave-tables in figure 6.8 into your application.
  // Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  // 5. In the main application loop, use an index variable to write the next value in the wave-table (array) 
  // to the appropriate DAC data register.
  while (1) {
    // SetLEDSByADC();
    int i;
    for (i = 0; i < 32; i++) {
      DAC1->DHR8R1 = sine_table[i];
      // 6. Use a 1ms delay between updating the DAC to new values.
      HAL_Delay(1);
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType =  RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
