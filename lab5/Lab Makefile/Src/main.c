/**
  *
  * Brandon Mouser
  * U0962682
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) 
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  /* 5.2 Setting the GPIO modes */
  // Enable GPIOB and GPIOC in the RCC.
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  
  // Enable I2C2 in the RCC.
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Initialize LEDs.
  GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                              GPIO_MODE_OUTPUT_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr);

  // Set PB11.
  // Alternate Function mode
  GPIOB->MODER |= GPIO_MODER_MODER11_1;
  GPIOB->MODER &= ~GPIO_MODER_MODER11_0;
  // Open-drain Output type
  GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
  // Select I2C2_SDA as its alternate function.
  GPIOB->AFR[1] |= (1 << 12);

  // Set PB13.
  // Alternate Function mode
  GPIOB->MODER |= GPIO_MODER_MODER13_1;
  GPIOB->MODER &= ~GPIO_MODER_MODER13_0;
  // Open-drain Output type
  GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
  // Select I2C2_SCL as its alternate function.
  GPIOB->AFR[1] |= (5 << 20);

  // Set PB14.
  // Output mode
  GPIOB->MODER &= ~GPIO_MODER_MODER14_1;
  GPIOB->MODER |= GPIO_MODER_MODER14_0;
  // Push-Pull output type
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;
  // Initialize/set the pin high.
  GPIOB->ODR |= GPIO_ODR_14;

  // Set PC0.
  // Output mode
  GPIOC->MODER &= ~GPIO_MODER_MODER0_1;
  GPIOC->MODER |= GPIO_MODER_MODER0_0;
  // Push-Pull Output type
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;
  // Initialize/set the pin high.
  GPIOC->ODR |= GPIO_ODR_0;

  // Set PB15.
  GPIOB->MODER &= ~(GPIO_MODER_MODER15_1 | GPIO_MODER_MODER15_0);

  /* 5.3 Initializing the I2C Peripheral */
  // Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.
  I2C2->TIMINGR |= (1 << 28);     // PRESC
  I2C2->TIMINGR |= (0x13 << 0);   // SCLL (0x13 == 0b10011)
  I2C2->TIMINGR |= (0xF << 8);    // SCLH (0xF == 0b1111)
  I2C2->TIMINGR |= (0x2 << 16);   // SDADEL (0x2 == 0b0010)
  I2C2->TIMINGR |= (0x4 << 20);   // SCADEL (0x4 == 0b0100)

  // Enable the I2C peripheral using the PE bit in the CR1 register.
  I2C2->CR1 |= I2C_CR1_PE;

  /* 5.4 Reading the Register */
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  // Set the transaction parameters in the CR2 register. (Do not set the AUTOEND bit.)
    // 1. Set the slave address in the SADD[7:1] bit field.
    I2C2->CR2 |= (0x69 << 1);
    // 2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
    I2C2->CR2 |= (1 << 16);
    // 3. Configure the RD_WRN to indicate a read/write operation.
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;
    // 4. Setting the START bit to begin the address frame.
    I2C2->CR2 |= I2C_CR2_START;

  // Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
  while (!(I2C2->ISR & I2C_ISR_TXIS))
  {
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }
  } 


  // Write the address of the “WHO_AM_I” register into the I2C transmit register. (TXDR)
  I2C2->TXDR |= (0x0F << 0);

  // Wait until the TC (Transfer Complete) flag is set.
  while (!(I2C2->ISR & I2C_ISR_TC));

  // Clear the bits.
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

  // 1. Set the slave address in the SADD[7:1] bit field.
  I2C2->CR2 |= (0x69 << 1);
  // 2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
  I2C2->CR2 |= (1 << 16);

  // Reload the CR2 register with the same parameters as before.
  // Set the RD_WRN bit to indicate a read operation & set the START bit again.
  I2C2->CR2 |= I2C_CR2_RD_WRN;
  I2C2->CR2 |= I2C_CR2_START;

  // Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set.
  while (!(I2C2->ISR & I2C_ISR_RXNE))
  {
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }
  } 

  // Wait until the TC (Transfer Complete) flag is set.
  while (!(I2C2->ISR & I2C_ISR_TC));

  // Check the contents of the RXDR register to see if it matches 0xD3. (expected value of the “WHO_AM_I” register)
  if (I2C2->RXDR == 0xD3)
  {
    // Turn on the blue LED.
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  }

  // Set the STOP bit in the CR2 register to release the I2C bus.
  I2C2->CR2 |= I2C_CR2_STOP;
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
