#include "SPI.h"

extern SPI_HandleTypeDef *ltc_spi;

bool SPIWrite(uint8_t *writeBuffer, uint8_t totalBytes)
{
  HAL_StatusTypeDef halReturnStatus;
  uint8_t readBuffer[255];

  HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);

  halReturnStatus = HAL_SPI_TransmitReceive(ltc_spi, writeBuffer, readBuffer, totalBytes, 1000U);

  while (ltc_spi->State == HAL_SPI_STATE_BUSY) {
  }

  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);

  return (halReturnStatus == HAL_OK);
}

bool SPIWriteRead(uint8_t *writeBuffer, uint8_t *readBuffer, uint8_t totalBytes)
{
  HAL_StatusTypeDef halReturnStatus;

  HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);

  halReturnStatus = HAL_SPI_TransmitReceive(ltc_spi, writeBuffer, readBuffer, totalBytes, 1000U);

  while (ltc_spi->State == HAL_SPI_STATE_BUSY) {
  }

  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);

  return (halReturnStatus == HAL_OK);
}
