
#include "SPI.h"
uint32_t write_block_start_time = 0, write_block_total_time = 0, rw_block_start_time = 0, rw_block_total_time = 0;
// MAX DELAY MAX_SPI_DELAY
bool SPIWrite(uint8_t *writeBuffer, uint8_t totalBytes) {

	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);	 // For debug only
	HAL_StatusTypeDef halReturnStatus;
	uint8_t readBuffer[totalBytes];

	HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);

	DISABLE_ALL_IRQS
	write_block_start_time = HAL_GetTick();

	halReturnStatus = HAL_SPI_TransmitReceive(ltc_spi, writeBuffer, readBuffer, totalBytes, MAX_SPI_DELAY);//1000);

	write_block_total_time = HAL_GetTick() - write_block_start_time;
	ENABLE_ALL_IRQS

	while (ltc_spi->State == HAL_SPI_STATE_BUSY)
	;  // wait xmission complete
	HAL_Delay(2);
	HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);

	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);  // For debug only
return (halReturnStatus == HAL_OK);
};

bool SPIWriteRead(uint8_t *writeBuffer, uint8_t *readBuffer, uint8_t totalBytes) {

	HAL_StatusTypeDef halReturnStatus;

	HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);

	DISABLE_ALL_IRQS
	rw_block_start_time = HAL_GetTick();

	halReturnStatus = HAL_SPI_TransmitReceive(ltc_spi, writeBuffer, readBuffer, totalBytes,  MAX_SPI_DELAY);//1000);

	rw_block_total_time = HAL_GetTick() - rw_block_start_time;
	ENABLE_ALL_IRQS

	while (ltc_spi->State == HAL_SPI_STATE_BUSY)
	;  // wait xmission complete

	HAL_Delay(2);
	HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);

	return (halReturnStatus == HAL_OK);
};
