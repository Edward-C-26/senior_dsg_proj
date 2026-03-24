
#include "PackCalculations.h"

//! @brief This method calculates the maximum and minimum cell voltages in
//!      the pack, and sets those values w/ the associated cell number to the
//!      bms critical info struct
//! @param cfg is the bms configuration file with constants used in our bms
//! @param bms is the bms struct that contains critical info regarding our pack 
//! @param bmsData is an array of NUM_CELLS cellData structs, containing index, fault,
//!      voltage and temperature
//! @returns none 
void setCriticalVoltages(BMS_critical_info_t *bms,
        CellData const bmsData[NUM_CELLS]) {
    uint16_t maxCellVoltage;
    uint16_t minCellVoltage;
    uint16_t cellVoltage;

    uint16_t totalVoltage = 0;


	bms->curr_min_voltage = MAXINT16;
	bms->curr_max_voltage = 0;

    for(uint8_t cell = 0; cell < NUM_CELLS; cell++) {
    	//First two is due to fucky cell voltages,
        //unsure about 0 -> for sume reason cellVoltage goes to 1027?

    	cellVoltage = bmsData[cell].voltage;

    	totalVoltage += (cellVoltage/1000);

    	maxCellVoltage = bms->curr_max_voltage;

        // Check if cell readings is a max voltage or min voltage
        if(cellVoltage > maxCellVoltage
                && cellVoltage < INVALID_VOLTAGE_UPPER_THRESHOLD) {
            bms->curr_max_voltage = cellVoltage;
			bms->max_volt_cell = cell;
        }
		if(cellVoltage >= INVALID_VOLTAGE_UPPER_THRESHOLD) {
			bms->invalid_data = true;
            bms->invalid_data_cell = cell;
		}

        minCellVoltage = bms->curr_min_voltage;

        if((cellVoltage < minCellVoltage || minCellVoltage == 0)
                && cellVoltage > INVALID_VOLTAGE_LOWER_THRESHOLD) {
            bms->curr_min_voltage = cellVoltage;
			bms->min_volt_cell = cell;
        }
		if(cellVoltage <= INVALID_VOLTAGE_LOWER_THRESHOLD) {
			bms->invalid_data = true;
            bms->invalid_data_cell = cell;
		}        
    }
	bms->cellMonitorPackVoltage = totalVoltage;
}


//! @brief This method calculates the maximum and minimum cell temps in the
//!      pack, and sets those values w/ the associated cell number to the bms
//!      critical info struct
//! @param cfg is the bms configuration file with constants used in our bms
//! @param bms is the bms struct that contains critical info regarding our pack 
//! @param bmsData is an array of NUM_CELLS cellData structs,
//!      containing index, fault, voltage and temperature
//! @returns none 
void setCriticalTemps(BMS_critical_info_t *bms, CellData const bmsData[NUM_CELLS]) {
	 	uint16_t maxCellTemp;
	    uint16_t minCellTemp;
	    uint16_t cellTemp;

		bms->curr_min_temp = MAXINT16;
		bms->curr_max_temp = 0;
	
	    for(uint8_t cell = 0; cell < NUM_CELLS; cell++) {
	    	cellTemp = bmsData[cell].temperature;
	    	maxCellTemp = bms->curr_max_temp;
	    	if(cellTemp > maxCellTemp){
	    	   bms->curr_max_temp = cellTemp;
	    	   bms->max_temp_cell = cell;
	    	}

	    	minCellTemp = bms->curr_min_temp;
	        if(cellTemp < minCellTemp || minCellTemp == 0) {
	            bms->curr_min_temp = cellTemp;
	            bms->min_temp_cell = cell;
	        }
	    }

}

//! @brief This function is still a work in progress
void balance(BMSConfigStructTypedef const *cfg, BMS_critical_info_t *bms,
        CellData bmsData[NUM_CELLS], bool cellDischarge[NUM_BOARDS][12],
        bool fullDischarge[NUM_BOARDS][12], uint8_t balanceCounter,
        uint8_t *chargeRate) {
	uint16_t maxCellVoltage = bms->curr_max_voltage;
//	uint16_t minCellVoltage = bms.curr_min_voltage;
	uint16_t cellVoltage = 0;
	int32_t differenceVoltage = 0;

    // if cell voltage in range, balance cells
	if ((maxCellVoltage > cfg->balancing_start_threshold)) {  
		for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {

			cellVoltage = bmsData[cell].voltage;

			differenceVoltage = (int32_t) cellVoltage - cfg->balancing_start_threshold;

            //For case where cell is not a "Top" (maximum)
			if(differenceVoltage < 0) continue;	
			
            uint8_t moduleIndex = cell / cfg->numOfCellsPerIC;
            uint8_t cellIndex = cell % cfg->numOfCellsPerIC;
			/*Unsure if this will work right now-- overall, structure is good,
             * but actual balancing seems weird*/
			if (cellVoltage > cfg->stopCharge_threshold 
                    && cellVoltage != 65535) {
            // if voltage is above charge threshold(and not a garbage value), 
            // turn off charger
				*chargeRate = 0;
				fullDischarge[moduleIndex][cellIndex] = 1;
			} else if (cell % 12 == balanceCounter 
                    && differenceVoltage > cfg->balancing_difference) {
				cellDischarge[moduleIndex][cellIndex] = 1;

			} else {
				cellDischarge[moduleIndex][cellIndex] = 0;
			}
		}
	}
	
	balance_counter++;

    if(balance_counter >= 12) {
        balance_counter = 0;
    } 
}

//! @brief WIP/untested. Should be run while accumulator is not charging, discharges any cells above the threshold
void thresholdBalance(BMSConfigStructTypedef *cfg, BMS_critical_info_t *bms,
        CellData bmsData[NUM_CELLS], bool cell_discharge[NUM_BOARDS][12],
        uint16_t cell_discharge_threshold,
        uint8_t num_cells_discharge_per_secondary) {
	uint16_t cell_voltage = 0;

	if (num_cells_discharge_per_secondary == 0) {	// protect from divide by 0 error
		return;
	}

	uint8_t discharge_cell_modulo = 12 / num_cells_discharge_per_secondary;

	if (bms->curr_max_voltage > cell_discharge_threshold) { // if all cells are below the threshold, no need to discharge any of them
		for  (uint8_t cell = 0; cell < NUM_CELLS; cell++) {
			cell_voltage = bmsData[cell].voltage;

			if ((cell_voltage <= cell_discharge_threshold) || (cell_voltage == 65535)) {
				cell_discharge[cell / cfg->numOfCellsPerIC][cell % cfg->numOfCellsPerIC] = 0;
			}
			else if (cell % discharge_cell_modulo == balance_counter) { // We won't get to this point if the cell voltage is below the threshold
				cell_discharge[cell / cfg->numOfCellsPerIC][cell % cfg->numOfCellsPerIC] = 1;
			}
			else {
				cell_discharge[cell / cfg->numOfCellsPerIC][cell % cfg->numOfCellsPerIC] = 0;
			}
		}

		balance_counter++;

		if(balance_counter >= discharge_cell_modulo) {
			balance_counter = 0;
		}
	}
}
