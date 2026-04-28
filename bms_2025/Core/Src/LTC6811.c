
#include "LTC6811.h"

#include <stddef.h>
#include <string.h>

// boards will NOT be fucked in 2025 :pray: :pray:
#define BOARD_IS_FUCKED 0 

#define PEC_TABLE_SIZE 256
#define UNDER_TEMP_LIMIT 10
#define OVER_TEMP_LIMIT 55

uint16_t pec15Table[PEC_TABLE_SIZE];	   // Packet Error Code
uint16_t CRC15_POLY = 0x4599;  // Explain magic number por favor -> In datasheet :)



//! @brief Initializes Packet Error Code LUT by generating PEC look up table -> call on startup
//! @returns none
void initPECTable(void) {
    uint16_t remainder;

    for (uint16_t i = 0; i < PEC_TABLE_SIZE; i++) {
        remainder = i << 7;
        for (int bit = 8; bit > 0; bit--) {
            if (remainder & 0x4000) {
                remainder = ((remainder << 1));
                remainder = (remainder ^ CRC15_POLY);
            } else {
                remainder = ((remainder << 1));
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
}

//! @brief This method is used when sending a command to calculate the necessary PEC bytes to follow command bytes. 
//!		Should be used when receiving data to compare receieved PEC with expected PEC value
//! @param len is the number of bytes that we will be stepping through in our data 
//! @param data is the data that we will be using to calculate the expected PEC
//! @returns the expected PEC value
uint16_t calculatePEC(uint8_t len, uint8_t *data) {	 // changed to take data by value now that we are not using malloc (ensure no edit)
	uint16_t remainder, address;
	remainder = 16;	 // PEC seed

	for (uint8_t i = 0; i < len; i++) {
		address = ((remainder >> 7) ^ data[i]) & 0xFF;	// calculate PEC table address
		remainder = (remainder << 8) ^ pec15Table[address];
	}

	return (remainder * 2);	 // The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

//! @brief this method is used for cellDischarge in order to write configuration datra to the LTC with specified address
//! @param cfg is the configuration struct for BMS constants 
//! @param address is the address used to pass into our config file 
//! @returns none 
void writeConfigAddress(BMSConfigStructTypedef *cfg, uint8_t address) {
//	uint8_t config[6];
	uint8_t cmd[12];
	uint16_t PEC_return;
	uint8_t dummy[8];

//	config[0] = (uint8_t)((cfg->GPIO5PulldownOff << 7) | (cfg->GPIO4PulldownOff << 6) | (cfg->GPIO3PulldownOff << 5) | (cfg->GPIO2PulldownOff << 4) | (cfg->GPIO1PulldownOff << 3) | (cfg->ReferenceOn << 2) | (cfg->ADCModeOption));
//	config[1] = (uint8_t)(cfg->UndervoltageComparisonVoltage & 0xFF);
//	config[2] = (uint8_t)(((cfg->OvervoltageComparisonVoltage << 4) & 0xF0) | ((cfg->UndervoltageComparisonVoltage >> 8) & 0x0F));
//	config[3] = (uint8_t)((cfg->OvervoltageComparisonVoltage >> 4) & 0xFF);
//	config[4] = (uint8_t)((cfg->DischargeCell[7] << 7) | (cfg->DischargeCell[6] << 6) | (cfg->DischargeCell[5] << 5) | (cfg->DischargeCell[4] << 4) | (cfg->DischargeCell[3] << 3) | (cfg->DischargeCell[2] << 2) | (cfg->DischargeCell[1] << 1) | (cfg->DischargeCell[0]));
//	config[5] = (uint8_t)(((cfg->DischargeTimeoutValue << 4) & 0xF0) | (cfg->DischargeCell[11] << 3) | (cfg->DischargeCell[10] << 2) | (cfg->DischargeCell[9] << 1) | (cfg->DischargeCell[8]));

	cmd[0] = (uint8_t)((0x80 | ((address << 3) & 0x78) | ((WriteConfigurationRegisterGroup >> 8) & 0x07)));
	cmd[1] = (uint8_t)(WriteConfigurationRegisterGroup & 0xFF);

	PEC_return = calculatePEC(2, cmd);

	cmd[2] = (uint8_t)((PEC_return >> 8) & 0xFF);
	cmd[3] = (uint8_t)(PEC_return & 0xFF);

//	memcpy(&cmd[4], config, 6 * sizeof(config[0]));

	cmd[4] = (uint8_t)((cfg->GPIO5PulldownOff << 7) | (cfg->GPIO4PulldownOff << 6) | (cfg->GPIO3PulldownOff << 5) | (cfg->GPIO2PulldownOff << 4) | (cfg->GPIO1PulldownOff << 3) | (cfg->ReferenceOn << 2) | (cfg->ADCModeOption));
	cmd[5] = (uint8_t)(cfg->UndervoltageComparisonVoltage & 0xFF);
	cmd[6] = (uint8_t)(((cfg->OvervoltageComparisonVoltage << 4) & 0xF0) | ((cfg->UndervoltageComparisonVoltage >> 8) & 0x0F));
	cmd[7] = (uint8_t)((cfg->OvervoltageComparisonVoltage >> 4) & 0xFF);
	cmd[8] = (uint8_t)((cfg->DischargeCell[7] << 7) | (cfg->DischargeCell[6] << 6) | (cfg->DischargeCell[5] << 5) | (cfg->DischargeCell[4] << 4) | (cfg->DischargeCell[3] << 3) | (cfg->DischargeCell[2] << 2) | (cfg->DischargeCell[1] << 1) | (cfg->DischargeCell[0]));
	cmd[9] = (uint8_t)(((cfg->DischargeTimeoutValue << 4) & 0xF0) | (cfg->DischargeCell[11] << 3) | (cfg->DischargeCell[10] << 2) | (cfg->DischargeCell[9] << 1) | (cfg->DischargeCell[8]));

	PEC_return = calculatePEC(6, cmd + 4);

	cmd[10] = (uint8_t)((PEC_return >> 8) & 0xFF);
	cmd[11] = (uint8_t)(PEC_return & 0xFF);

	SPIWrite(cmd, sizeof(cmd));

	readConfig(address, dummy);

}

//! @brief This function is called every loop to accommodate dischargeCells method 
//!		Specifically writes configuration data (UV, OV, ADCMode, etc) to BMSConfig struct
//! @param cfg is the configuration files for the constants in the BMS system
//! @returns none
void writeConfigAll(BMSConfigStructTypedef *cfg) {
	wakeup_idle();

	for (uint8_t i = 0; i < cfg->numOfICs; i++) {
		writeConfigAddress(cfg, cfg->address[i]);
	}
}

//! @brief This function sends an ADCV command that begins conversion for every cell to specified LTC. 
//!		This results in the function readings all cell voltage registers using the readRegister function. 
//! @param address is the address of the board that will be read 
//! @param cellVoltage is the array that will store the voltages of all the cells on the board being read
//! @returns true if PEC matches expected PEC value, else false 
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]) {
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t voltage[12];
	//board 11 is fucked 
	if (BOARD_IS_FUCKED && address == 11){
		PEC_check = readRegister(ReadCellVoltageRegisterGroup1to3, address, voltage);
		dataValid = dataValid & PEC_check;

		PEC_check = readRegister(ReadCellVoltageRegisterGroup4to6, address, voltage);
		dataValid = dataValid & PEC_check;

		PEC_check = readRegister(ReadCellVoltageRegisterGroup7to9, address, voltage);
		dataValid = dataValid & PEC_check;
		
		PEC_check = readRegister(ReadCellVoltageRegisterGroup10to12, address, voltage);
		dataValid = dataValid & PEC_check;


		//THIS IS TERRIBLE PRACTICE BE CAREFUL
		voltage[1] = voltage[5];
//		voltage[1] = 44000;

		voltage[2] = voltage[5];
		voltage[11] = voltage[5];


	} else {
	
		PEC_check = readRegister(ReadCellVoltageRegisterGroup1to3, address, voltage);
		dataValid = dataValid & PEC_check;

		PEC_check = readRegister(ReadCellVoltageRegisterGroup4to6, address, voltage);
		dataValid = dataValid & PEC_check;

		PEC_check = readRegister(ReadCellVoltageRegisterGroup7to9, address, voltage);
		dataValid = dataValid & PEC_check;

		PEC_check = readRegister(ReadCellVoltageRegisterGroup10to12, address, voltage);
		dataValid = dataValid & PEC_check;
	}

	memcpy(cellVoltage, voltage, sizeof(voltage));

	return dataValid;
}

//! @brief This function reads all cell voltages by essentially parsing each board and reading the individual cell voltages. These are then 
//!		stored in the bmsData array, along with the cell number that is associated with the reading. 
//! @param bmsData is an array of 144 cellData structs, containing index, fault, voltage and temperature
//! @returns true if no PEC for any register read for any board 
bool readAllCellVoltages(CellData bmsData[]) {
	// DEAR WHOEVER READS/ RUNS THIS CODE
	// DO NOT BREAKPOINT HERE UNLESS YOU WANT TO GOOF TIMING UP
	uint16_t boardVoltage[12];
	bool PEC_check[12];
	bool dataValid = true;

	wakeup_idle();
	HAL_Delay(2);

	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartCellVoltageADCConversionAll);
	HAL_Delay(10);

	wakeup_idle();
	HAL_Delay(2);

//	uint16_t allVoltages[144];


	for (uint8_t board = 0; board < NUM_BOARDS; board++) {

		// read voltage of every cell input (1-12) for a specific address, store in boardVoltage
		PEC_check[board] = readCellVoltage(board, boardVoltage);


		dataValid &= PEC_check[board];

		// store cell number and valid data bit in bmsData
		for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {
			const uint8_t cell_index = (uint8_t)((board * 12U) + cell);
			if (cell_index >= NUM_CELLS) {
				break;
			}
			bmsData[cell_index].voltage = (uint8_t)(cell_index + 1U);  // cell number

			if (!PEC_check[board]) {
				bmsData[cell_index].fault |= CELL_PEC_FAIL_MASK;
            } else {
				bmsData[cell_index].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            }

			bmsData[cell_index].voltage = boardVoltage[cell];
		}
	}


	return dataValid;  // return true if no PEC errors for any board
}

//! @brief This function initiates ADC conversion for GPIO inputs connected to temperature sensors. 
//!		Reads auxiliary register groups using readRegister function. Then, converts measured voltage into temperature 
//!			based on temperature sensor response. Also checks for disconnected temperature sensor and OT faults. 
//!				NOTE: We only read 4 temps per board, and they are the 4 highest temps on each board.
//! @param address is the address of the board being read
//! @param cellTemp is the array that will hold the cell temps read during readRegister
//! @param dcFault is the array that stores temperature sensor ceonnection for all cells read 
//! @param tempFault stores OT fault info for each cell read 
//! @returns true if PEC value read matches expected PEC value calculated. Otherwise, false. 
bool readCellTemp(uint8_t address, uint16_t cellTemp[4], bool dcFault[4], bool tempFault[4]) {
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t temp[4];
//	double realTemp[4];

	for (uint8_t i = 0; i < 4; i++) {
		cellTemp[i] = 0U;
		dcFault[i] = true;
		tempFault[i] = false;
		temp[i] = 0;
//		realTemp[i] = 0.0;
	}


	//30 to 50c 

	PEC_check = readRegister(ReadAuxiliaryGroupA, address, temp);
	dataValid = dataValid & PEC_check;

	PEC_check = readRegister(ReadAuxiliaryGroupB, address, temp);
	dataValid = dataValid & PEC_check;

	//float minimalVoltage = 0.2083; //will remove -> this is the lowest temp we can read that we care about

	for (uint8_t i = 0; i < 4; i++){
			// uint8_t truncatedADC = ((cellTemp[i] >> 6) << 6); //remove 8 most lsb bits (since ADC is 12 bit resolution this is actually removing the 2 most LSB) from the adc reading to get eventual 3 digit resolution
			double first = temp[i]/100;
			uint16_t second = (uint16_t)round(first);
			int index = second - 21;//convert adc to integer
			if ((index < 0) || (index >= (int)(sizeof(lookupTableTemps) / sizeof(lookupTableTemps[0])))) {
				cellTemp[i] = 0U;
				dcFault[i] = true;
				tempFault[i] = false;
				dataValid = false;
				continue;
			}

            uint16_t lookupVal = (uint16_t)lookupTableTemps[index];
			cellTemp[i] =  lookupVal * 1000U;

			dcFault[i] = false;
			tempFault[i] = ((lookupVal < UNDER_TEMP_LIMIT)
                    || OVER_TEMP_LIMIT < lookupVal) ? true : false;
		
	}

		return (dataValid);

}

//! @brief This function reads cell temps from all of our board by calling readCellTemp for each board. NOTE : This is not proven to work properly yet.
//! @param bmsData is an array of 144 cellData structs, containing index, fault, voltage and temperature
//! @returns true if PEC value received matches expected PEC value. Otherwise, false. -> i.e., returns false if any readCellTemp return values are false. 
bool readAllCellTemps(CellData bmsData[]) {
	uint16_t boardTemp[4];
	bool boardDCFault[4];
	bool boardTempFault[4];
	bool PEC_check[12];
	bool dataValid = true;

	wakeup_idle();
	HAL_Delay(2);

	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartCellTempVoltageADCConversionAll);
	HAL_Delay(20);

	wakeup_idle();
	HAL_Delay(2);

	for (uint8_t board = 0; board < NUM_BOARDS; board++) {
		// read temperature, check for OT and temp DC
		PEC_check[board] = readCellTemp(board, boardTemp, boardDCFault, boardTempFault);
		dataValid &= PEC_check[board];

		// store OT and temp DC bits in status byte
		for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {
			const uint8_t cell_index = (uint8_t)((board * 12U) + cell);
			if (cell_index >= NUM_CELLS) {
				break;
			}

			if (!PEC_check[board]) {
				bmsData[cell_index].fault |= CELL_PEC_FAIL_MASK;
            } else {
				bmsData[cell_index].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            }

			if (boardTempFault[cell / 3U]) {
				bmsData[cell_index].fault |= CELL_TEMP_FAIL_MASK;
            } else {
				bmsData[cell_index].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);	// set OT bit
            }

			if (boardDCFault[cell / 3U]) {
				bmsData[cell_index].fault |= CELL_DCFAULT_MASK;
            } else {
				bmsData[cell_index].fault &= (uint8_t)(~CELL_DCFAULT_MASK);
            }

			bmsData[cell_index].temperature = boardTemp[cell / 3U];
		}
	}

	return dataValid;
}

//! @brief This method uses general readRegister function to check current state of LTC configuration reg. 
//!		Mostly used for testing purposes
//! @param address is the address passed into the readRegister function
//! @param cfg is part of the configuration that will be passed to check the current state of the LTC
//! @returns true if the received PEC from readRegister mathes the calculated PEC 
bool readConfig(uint8_t address, uint8_t cfg[8]) {
	uint16_t config[4];
	bool dataValid = false;

	wakeup_idle();

	dataValid = readRegister(ReadConfigurationRegisterGroup, address, config);

	cfg[0] = (uint8_t)((config[0] >> 8) & 0xFF);
	cfg[1] = (uint8_t)(config[0] & 0xFF);
	cfg[2] = (uint8_t)((config[1] >> 8) & 0xFF);
	cfg[3] = (uint8_t)(config[1] & 0xFF);
	cfg[4] = (uint8_t)((config[2] >> 8) & 0xFF);
	cfg[5] = (uint8_t)(config[2] & 0xFF);
	cfg[6] = (uint8_t)((config[3] >> 8) & 0xFF);
	cfg[7] = (uint8_t)(config[3] & 0xFF);

	return dataValid;
}

//! @brief This function checks the cell connections of each cell in the BMS data array.
//!		Currently not used, as we have not been able to get it working, but the whole idea is that 
//!			this function compares previously measured values to open wire check values, and if 
//!				there is a significant drop in voltage, cell is allegedly disconnected. 
//! @param cfg is the BMS configuration struct with constants 
//! @param bmsData is an array of 144 cellData structs, containing index, fault, voltage and temperature
//! @returns true is the cell is connnected properly, and false if the cell is "disconnected"
bool checkAllCellConnections(BMSConfigStructTypedef cfg, CellData bmsData[]) {
	uint16_t ADOWvoltage[cfg.numOfCellInputs];
	uint16_t cellVoltage;
	bool PEC_check[12];
	bool dataValid = true;

	wakeup_idle();

	// at least 2
	// por que los dos
	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	HAL_Delay(20);

	wakeup_idle();

	for (uint8_t board = 0; board < cfg.numOfICs; board++) {
		PEC_check[board] = readCellVoltage(cfg.address[board], ADOWvoltage);
		dataValid &= PEC_check[board];

		// I'm not touching this but wtf
		ADOWvoltage[4] = ADOWvoltage[6];
		ADOWvoltage[5] = ADOWvoltage[7];
		ADOWvoltage[6] = ADOWvoltage[8];
		ADOWvoltage[7] = ADOWvoltage[9];

		for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
			cellVoltage = bmsData[(board * cfg.numOfCellsPerIC) + cell].voltage;

			if (!((cellVoltage - ADOWvoltage[cell]) < 1000)) {
                bmsData[(board * cfg.numOfCellsPerIC) + cell].fault |= CELL_DISCONNECT_MASK;
			} else {
                bmsData[(board * cfg.numOfCellsPerIC) + cell].fault &= (uint8_t)(~CELL_DISCONNECT_MASK);
            }
		}
	}

	return dataValid;
}

//! @brief This function writes the configuration struct of cells that are currently being discharged. This is to track which cells are //		being discharged while running our balancing algorithm. 
//! @param cfg is the configuration struct that stores all the constants in our BMS algorithm 
//! @param cellDischarge is a 2D array of cells that tracks which cells are being discharged or not : 1 = discharging, 0 = not discharging 
//! returns 0 always -> not sure why we need a return value on this 
bool dischargeCellGroups(BMSConfigStructTypedef *cfg, bool cellDischarge[12][12]) {
	wakeup_idle();

	for (uint8_t i = 0; i < cfg->numOfICs; i++) {
		for (uint8_t j = 0; j < 12; j++) {
			cfg->DischargeCell[j] = cellDischarge[i][j];
		}

		writeConfigAddress(cfg, cfg->address[i]);
	}

	for (uint8_t i = 0; i < 12; i++)
		cfg->DischargeCell[i] = 0;

	return 0;
}

//! @brief This function is used to wakeup the LTC chip that we want to use to get readings from 
void inline wakeup_idle() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(1);
}

bool poll_single_secondary_voltage_reading(uint8_t board_num, BMSConfigStructTypedef *cfg, CellData bmsData[]){
	uint16_t boardVoltage[12];
	bool PEC_check[12]	;
	bool dataValid = true;

	memset(boardVoltage, 0, sizeof(boardVoltage));
	memset(PEC_check, 0, sizeof(PEC_check));

	wakeup_idle();
	writeConfigAddress(cfg, cfg->address[board_num]);

	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartCellVoltageADCConversionAll);

	HAL_Delay(3);
	wakeup_idle();
	HAL_Delay(1);

	// read voltage of every cell input (1-12) for a specific address, store in boardVoltage
	PEC_check[board_num] = readCellVoltage(board_num, boardVoltage);

	dataValid &= PEC_check[board_num];

	// store cell number and valid data bit in bmsData
	for (uint8_t cell = 0; cell < cfg->numOfCellInputs; cell++) {
		const uint8_t cell_index = (uint8_t)((board_num * 12U) + cell);
		if (cell_index >= NUM_CELLS) {
			break;
		}
		bmsData[cell_index].voltage = (uint8_t)(cell_index + 1U);  // cell number

		if (!PEC_check[board_num]) {
			bmsData[cell_index].fault |= CELL_PEC_FAIL_MASK;
		} else {
			bmsData[cell_index].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
		}

		bmsData[cell_index].voltage = boardVoltage[cell];
	}

	return dataValid;

}

bool poll_single_secondary_temp_reading(uint8_t board_num, BMSConfigStructTypedef *cfg, CellData bmsData[]){

	// do writeConfigAll(&BMSConfig);
	uint16_t boardTemp[4];
	bool boardDCFault[4];
	bool boardTempFault[4];
	bool PEC_check[12];
	bool dataValid = true;

	memset(boardTemp, 0, sizeof(boardTemp));
	memset(boardDCFault, 1, sizeof(boardDCFault));
	memset(boardTempFault, 0, sizeof(boardTempFault));
	memset(PEC_check, 0, sizeof(PEC_check));
	//Do write config all
	wakeup_idle();

	writeConfigAddress(cfg, cfg->address[board_num]);

	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartCellTempVoltageADCConversionAll);

	HAL_Delay(3);


	wakeup_idle();
	HAL_Delay(1);

	// read temperature, check for OT and temp DC
	PEC_check[board_num] = readCellTemp(board_num, boardTemp, boardDCFault, boardTempFault);

	dataValid &= PEC_check[board_num];

	// store OT and temp DC bits in status byte
	for (uint8_t cell = 0; cell < cfg->numOfCellsPerIC; cell++) {
		const uint8_t cell_index = (uint8_t)((board_num * 12U) + cell);
		if (cell_index >= NUM_CELLS) {
			break;
		}

		if (!PEC_check[board_num]) {
			bmsData[cell_index].fault |= CELL_PEC_FAIL_MASK;
		} else {
			bmsData[cell_index].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
		}

		if (boardTempFault[cell / 3U]) {
			bmsData[cell_index].fault |= CELL_TEMP_FAIL_MASK;
		} else {
			bmsData[cell_index].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);	// set OT bit
		}

		if (boardDCFault[cell / 3U]) {
			bmsData[cell_index].fault |= CELL_DCFAULT_MASK;
		} else {
			bmsData[cell_index].fault &= (uint8_t)(~CELL_DCFAULT_MASK);
		}

		bmsData[cell_index].temperature = boardTemp[cell / 3U];
	}

	return dataValid;

}

//! @brief Reads register specified by command from specified board address 
//! @param command is the command that we will send to the board
//! @param address is the specific board that will get the command
//! @param data is where the data read from the specific board will be StartOpenWireConversionPulldown
//! @returns true if the data is valid, otherwise false (i.e., received PEC matches expected PEC value)
bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data) {  // changed to take array by reference now that malloc is removed
	uint8_t cmd[12];
	uint8_t rx_data[12];
	uint16_t PEC_return;
	uint8_t PEC_send[6];
	bool dataValid = true;

	memset(rx_data, 0, sizeof(rx_data));

//	if (data != NULL) {
//		size_t words_to_clear = 0U;
//
//		switch (command) {
//			case ReadCellVoltageRegisterGroup1to3:
//			case ReadCellVoltageRegisterGroup4to6:
//			case ReadCellVoltageRegisterGroup7to9:
//			case ReadCellVoltageRegisterGroup10to12:
//				words_to_clear = 12U;
//				break;
//			case ReadAuxiliaryGroupA:
//			case ReadAuxiliaryGroupB:
//				words_to_clear = 4U;
//				break;
//			case ReadConfigurationRegisterGroup:
//				words_to_clear = 4U;
//				break;
//			default:
//				words_to_clear = 0U;
//				break;
//		}
//
//		if (words_to_clear > 0U) {
//			memset(data, 0, words_to_clear * sizeof(*data));
//		}
//	}
//
	PEC_send[0] = (uint8_t)(0x80 | ((address << 3) & 0x78) | ((command >> 8) & 0x07));
	PEC_send[1] = (uint8_t)(command & 0xFF);

	cmd[0] = PEC_send[0];
	cmd[1] = PEC_send[1];

	PEC_return = calculatePEC(2, PEC_send);

	cmd[2] = (uint8_t)((PEC_return >> 8) & 0xFF);
	cmd[3] = (uint8_t)(PEC_return & 0xFF);

	cmd[4] = 0;
	cmd[5] = 0;
	cmd[6] = 0;
	cmd[7] = 0;
	cmd[8] = 0;
	cmd[9] = 0;
	cmd[10] = 0;
	cmd[11] = 0;

	if (!SPIWriteRead(cmd, rx_data, sizeof(cmd))) {  // send 4 command bytes, receive 6 cell voltage bytes (4-9) and 2 PEC bytes (10-11)
		return false;
	}

	// calculate PEC based on cell voltage data received
	PEC_send[0] = rx_data[4];  // cell 1 voltage low bytes
	PEC_send[1] = rx_data[5];  // cell 1 voltage high bytes
	PEC_send[2] = rx_data[6];  // cell 2 voltage low bytes
	PEC_send[3] = rx_data[7];  // cell 2 voltage high bytes
	PEC_send[4] = rx_data[8];  // cell 3 voltage low bytes
	PEC_send[5] = rx_data[9];  // cell 3 voltage high bytes

	PEC_return = calculatePEC(6, PEC_send);

	// check if received PEC matches calculated PEC
	if (PEC_return != (((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}

	if (command == ReadCellVoltageRegisterGroup1to3) {
		data[0] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[1] = (uint16_t)((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[2] = (uint16_t)((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
		// data[3] = (uint16_t) ((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF);
		// data[4] = PEC_return;
	}

	if (command == ReadCellVoltageRegisterGroup4to6) {
		data[3] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[4] = (uint16_t)((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[5] = (uint16_t)((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}

	if (command == ReadCellVoltageRegisterGroup7to9) {
		data[6] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[7] = (uint16_t)((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[8] = (uint16_t)((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}

	if (command == ReadCellVoltageRegisterGroup10to12) {
		data[9] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[10] = (uint16_t)((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[11] = (uint16_t)((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}

	if (command == ReadAuxiliaryGroupA) {
		data[0] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[1] = (uint16_t)((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[2] = (uint16_t)((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}

	if (command == ReadAuxiliaryGroupB) {
		data[3] = (uint16_t)((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
	}

	if (command == ReadConfigurationRegisterGroup) {
		data[0] = (uint16_t)((rx_data[4] << 8) & 0xFF00) | (rx_data[5] & 0x00FF);
		data[1] = (uint16_t)((rx_data[6] << 8) & 0xFF00) | (rx_data[7] & 0x00FF);
		data[2] = (uint16_t)((rx_data[8] << 8) & 0xFF00) | (rx_data[9] & 0x00FF);
		data[3] = (uint16_t)((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF);
	}

	return (dataValid);
}

//! @brief Sends specified write only command to every LTC in the chain (ex: ADCV)
//! @param command contains every command code used 
//! @returns none 
void sendBroadcastCommand(CommandCodeTypedef command) {
	uint8_t cmd[4];
	uint16_t PEC_return;

	cmd[0] = (uint8_t)((command >> 8) & 0x0F);
	cmd[1] = (uint8_t)(command & 0xFF);

	PEC_return = calculatePEC(2, (uint8_t *)&(cmd));

	cmd[2] = (uint8_t)((PEC_return >> 8) & 0xFF);
	cmd[3] = (uint8_t)(PEC_return & 0xFF);

	SPIWrite(cmd, 4);
}

//! @brief Sends specified write-only command to LTC with the specified address 
//! @param command is the command that will be sent to the LTC 
//! @param address is the specified address of the LTC that will receive the command
//! @returns none 
void sendAddressCommand(CommandCodeTypedef command, uint8_t address) {
	uint8_t cmd[4];
	uint16_t PEC_return;
	uint8_t msbytes[2];

	msbytes[0] = (uint8_t)(0x80 | ((address << 3) & 0x78) | ((command >> 8) & 0x07));
	msbytes[1] = (uint8_t)(command & 0xFF);

	cmd[0] = msbytes[0];
	cmd[1] = msbytes[1];

	PEC_return = calculatePEC(2, msbytes);

	cmd[2] = (uint8_t)((PEC_return >> 8) & 0xFF);
	cmd[3] = (uint8_t)(PEC_return & 0xFF);

	SPIWrite(cmd, 4);
}
