/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BLE_PWRON_Pin GPIO_PIN_13
#define BLE_PWRON_GPIO_Port GPIOC
#define LED_A_Pin GPIO_PIN_0
#define LED_A_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOC
#define LED_C_Pin GPIO_PIN_2
#define LED_C_GPIO_Port GPIOC
#define LED_D_Pin GPIO_PIN_3
#define LED_D_GPIO_Port GPIOC
//////////////////////////////////////////
#define DW_IRQ_A_Pin GPIO_PIN_0
#define DW_IRQ_A_GPIO_Port GPIOA
#define DW_RST_B_Pin GPIO_PIN_1
#define DW_RST_B_GPIO_Port GPIOA
////////////////////////////////////////////
#define BLE_UART_TX_Pin GPIO_PIN_2
#define BLE_UART_TX_GPIO_Port GPIOA
#define BLE_UART_RX_Pin GPIO_PIN_3
#define BLE_UART_RX_GPIO_Port GPIOA

//////////////////////////////////
#define DW_CS_A_Pin GPIO_PIN_4
#define DW_CS_A_GPIO_Port GPIOA
#define DW_CLK_A_Pin GPIO_PIN_5
#define DW_CLK_A_GPIO_Port GPIOA
#define DW_MISO_A_Pin GPIO_PIN_6
#define DW_MISO_A_GPIO_Port GPIOA
#define DW_MOSI_A_Pin GPIO_PIN_7
#define DW_MOSI_A_GPIO_Port GPIOA
#define DW_WUP_B_Pin GPIO_PIN_0
#define DW_WUP_B_GPIO_Port GPIOB
#define DW_WUP_A_Pin GPIO_PIN_1
#define DW_WUP_A_GPIO_Port GPIOB
//////////////////////////////////////
#define EXT_IO1_Pin GPIO_PIN_2
#define EXT_IO1_GPIO_Port GPIOB

#define SPI2_CS1_Pin GPIO_PIN_12
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_6
#define SPI2_CS2_GPIO_Port GPIOC

#define DW_RST_A_Pin GPIO_PIN_10
#define DW_RST_A_GPIO_Port GPIOB
#define DW_IRQ_B_Pin GPIO_PIN_11
#define DW_IRQ_B_GPIO_Port GPIOB

#define MEMS_INT1_Pin GPIO_PIN_7
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT2_Pin GPIO_PIN_8
#define MEMS_INT2_GPIO_Port GPIOC
#define MEMS_INT3_Pin GPIO_PIN_9
#define MEMS_INT3_GPIO_Port GPIOC
#define MEMS_DRDY_Pin GPIO_PIN_8
#define MEMS_DRDY_GPIO_Port GPIOA
#define MEMS_DEN_Pin GPIO_PIN_2
#define MEMS_DEN_GPIO_Port GPIOD

#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_DPVCC_Pin GPIO_PIN_10
#define USB_DPVCC_GPIO_Port GPIOA
///////////////////////////////////////////////
#define DW_CS_B_Pin GPIO_PIN_15
#define DW_CS_B_GPIO_Port GPIOA
#define DW_CLK_B_Pin GPIO_PIN_10
#define DW_CLK_B_GPIO_Port GPIOC
#define DW_MISO_B_Pin GPIO_PIN_11
#define DW_MISO_B_GPIO_Port GPIOC
#define DW_MOSI_B_Pin GPIO_PIN_12
#define DW_MOSI_B_GPIO_Port GPIOC
///////////////////////////////////////////////////
#define EXT_IO2_Pin GPIO_PIN_3
#define EXT_IO2_GPIO_Port GPIOB
#define EXT_IO3_Pin GPIO_PIN_4
#define EXT_IO3_GPIO_Port GPIOB
#define DW_SYNC_Pin GPIO_PIN_5
#define DW_SYNC_GPIO_Port GPIOB
#define DW_SYNC_CLR_Pin GPIO_PIN_8
#define DW_SYNC_CLR_GPIO_Port GPIOB
#define DW_SYNC_EN_Pin GPIO_PIN_9
#define DW_SYNC_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DW_IRQ_A_IRQn           EXTI0_IRQn
#define DW_IRQ_B_IRQn           EXTI15_10_IRQn

/* APPLICATION LEDS */
#define LED_STATIONARY_Port     LED_A_GPIO_Port
#define LED_STATIONARY_Pin      LED_A_Pin
#define LED_NODE_Port           LED_B_GPIO_Port
#define LED_NODE_Pin            LED_B_Pin
#define LED_USB_Port            LED_C_GPIO_Port
#define LED_USB_Pin             LED_C_Pin
#define LED_ERROR_Port          LED_D_GPIO_Port
#define LED_ERROR_Pin           LED_D_Pin

/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
