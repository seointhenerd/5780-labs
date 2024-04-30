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

void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
char LEDcolor, LEDmode;
int isLEDSet;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void TransmitChar(char c)
{
  while (!(USART3->ISR & USART_ISR_TXE)); // exits once the flag is set.

  // Write the character into the transmit data register.
  USART3->TDR = c;
}

void TransmitString(char* str)
{
  for (int i = 0; str[i] != '\0'; i++)
  {
    TransmitChar(str[i]);
  }
}

void ReceiveLED()
{
  // Check and wait on the USART status flag that indicates the transmit register is empty.
  if (USART3->ISR & USART_CR1_RXNEIE)
  {
    LEDcolor = USART3->RDR;

    switch(LEDcolor) {
    case 'r': // Red LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_6);
        break;
    case 'b': // Blue LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_7);
        break;
    case 'o': // Orange LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_8);
        break;
    case 'g': // Green LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_9);
        break;
    }

    if (LEDcolor != 'r' && LEDcolor != 'b' && LEDcolor != 'o' && LEDcolor != 'g')
    {
      TransmitString("Unknown Input\n");
    }
  }
}

void USART3_4_IRQHandler()
{
  if (!isLEDSet)
  {
    LEDcolor = USART3->RDR;
    isLEDSet = 1;
  }
  else
  {
    LEDmode = USART3->RDR;
    switch(LEDcolor) {
      case 'r': // Red LED
        if (LEDmode == '0') GPIOC->ODR &= ~GPIO_ODR_6;
        else if (LEDmode == '1') GPIOC->ODR |= GPIO_ODR_6;
        else if (LEDmode == '2') HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_6);
        break;
      case 'b': // Blue LED
        if (LEDmode == '0') GPIOC->ODR &= ~GPIO_ODR_7;
        else if (LEDmode == '1') GPIOC->ODR |= GPIO_ODR_7;
        else if (LEDmode == '2') HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_7);
        break;
      case 'o': // Orange LED
        if (LEDmode == '0') GPIOC->ODR &= ~GPIO_ODR_8;
        else if (LEDmode == '1') GPIOC->ODR |= GPIO_ODR_8;
        else if (LEDmode == '2') HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_8);
        break;
      case 'g': // Green LED
        if (LEDmode == '0') GPIOC->ODR &= ~GPIO_ODR_9;
        else if (LEDmode == '1') GPIOC->ODR |= GPIO_ODR_9;
        else if (LEDmode == '2') HAL_GPIO_TogglePin(GPIOC, GPIO_ODR_9);
        break;
    }
    isLEDSet = 0;
  }

  if (LEDcolor != 'r' && LEDcolor != 'b' && LEDcolor != 'o' && LEDcolor != 'g')
  {
    TransmitString("Unknown Input\n");
  }
}

/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) 
{
  SystemClock_Config(); // Configure the system clock

  unsigned int baud_rate = 115200;

  // Enable the system clock to the desired USART in the RCC peripheral.
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;   // Enable GPIOB clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;   // Enable GPIOC clock

  // 4.1 Preparing to use the USART
  // PB10 - USART3_TX, PB11 - USART3_RX
  // Set MODER to Alternate Function mode
  GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
  GPIOB->MODER &= ~(GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11);
  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);

  // Configure LED pins
  GPIOC->MODER |= (GPIO_MODER_MODER9_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_0);
  GPIOC->MODER &= ~(GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1);
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6);
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR6);
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);

  GPIOB->AFR[1] |= ((4 << 8) | (4 << 12));

  // Set the Baud rate for communication to be 115200 bits/second.
  USART3->BRR = (uint16_t)(HAL_RCC_GetHCLKFreq() / baud_rate); // 69
  USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);    // Enable Receiver
  USART3->CR1 |= USART_CR1_UE;                     // Enable USART
  USART3->CR1 |= USART_CR1_RXNEIE;


  // 4.3 Interrupt-Based Reception
  NVIC_EnableIRQ(USART3_4_IRQn);

  while(1)
  {
    HAL_Delay(1000);
    // ReceiveLED();
    TransmitString("CMD?");
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
