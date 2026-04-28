// Bridge firmware for STM32F103RB on Nucleo
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart1;  // ST-Link USART (to computer)
UART_HandleTypeDef huart2;  // CN3 USART (to F446)



void SystemClock_Config(void) {
  // HAL initialization and clock config
  HAL_Init();
}

void MX_USART1_UART_Init(void) {
  // USART1 (ST-Link): 115200 baud
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart1);
}

void MX_USART2_UART_Init(void) {
  // USART2 (CN3): same baud as your F446
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    // Data from F446 → forward to computer
	uint8_t data = *(huart->pRxBuffPtr);
    HAL_UART_Transmit(&huart1, &data, 1, 10);
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&data, 1);
  }
}

int main(void) {
  SystemClock_Config();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  
  uint8_t rx_data;
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  
  while (1) {
    // Bridge runs in interrupt handler
  }
}
