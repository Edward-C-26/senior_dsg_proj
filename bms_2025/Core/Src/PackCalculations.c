#include "PackCalculations.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#define PACK_TOTAL_VOLTAGE_DIVISOR_TO_MV   10U
#define INVALID_TEMP_READING               0U

static uint8_t s_threshold_balance_counter = 0U;

static void clear_discharge_matrix(bool discharge[NUM_BOARDS][12])
{
    for (uint8_t board = 0U; board < NUM_BOARDS; board++) {
        for (uint8_t cell = 0U; cell < 12U; cell++) {
            discharge[board][cell] = false;
        }
    }
}

void setCriticalVoltages(BMS_critical_info_t *bms, CellData const bmsData[NUM_CELLS])
{
    uint32_t total_pack_mV = 0U;
    bool valid_found = false;

    bms->curr_min_voltage = UINT16_MAX;
    bms->curr_max_voltage = 0U;
    bms->invalid_data = false;
    bms->invalid_data_cell = 255U;
    bms->min_volt_cell = 255U;
    bms->max_volt_cell = 255U;

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        const uint16_t cellVoltage = bmsData[cell].voltage;
        const bool valid_voltage =
            (cellVoltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cellVoltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            bms->invalid_data = true;
            bms->invalid_data_cell = cell;
            continue;
        }

        valid_found = true;

        /*
         * LTC6811 cell codes are typically in 100 uV/count.
         * 42000 -> 4.2000 V -> 4200 mV
         */
        total_pack_mV += (uint32_t)(cellVoltage / PACK_TOTAL_VOLTAGE_DIVISOR_TO_MV);

        if (cellVoltage > bms->curr_max_voltage) {
            bms->curr_max_voltage = cellVoltage;
            bms->max_volt_cell = cell;
        }

        if (cellVoltage < bms->curr_min_voltage) {
            bms->curr_min_voltage = cellVoltage;
            bms->min_volt_cell = cell;
        }
    }

    if (!valid_found) {
        bms->curr_min_voltage = 0U;
        bms->curr_max_voltage = 0U;
        bms->min_volt_cell = 255U;
        bms->max_volt_cell = 255U;
        bms->cellMonitorPackVoltage = 0U;
        return;
    }

    if (total_pack_mV > UINT16_MAX) {
        bms->cellMonitorPackVoltage = UINT16_MAX;
    } else {
        bms->cellMonitorPackVoltage = (uint16_t)total_pack_mV;
    }
}

void setCriticalTemps(BMS_critical_info_t *bms, CellData const bmsData[NUM_CELLS])
{
    bool valid_found = false;

    bms->curr_min_temp = UINT16_MAX;
    bms->curr_max_temp = 0U;
    bms->min_temp_cell = 255U;
    bms->max_temp_cell = 255U;

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        const uint16_t cellTemp = bmsData[cell].temperature;

        /* Do not treat 0C as automatically invalid */
        valid_found = true;

        if (cellTemp > bms->curr_max_temp) {
            bms->curr_max_temp = cellTemp;
            bms->max_temp_cell = cell;
        }

        if (cellTemp < bms->curr_min_temp) {
            bms->curr_min_temp = cellTemp;
            bms->min_temp_cell = cell;
        }
    }

    if (!valid_found) {
        bms->curr_min_temp = 0U;
        bms->curr_max_temp = 0U;
        bms->min_temp_cell = 255U;
        bms->max_temp_cell = 255U;
    }
}

void balance(BMSConfigStructTypedef const *cfg,
             BMS_critical_info_t *bms,
             CellData bmsData[NUM_CELLS],
             bool cellDischarge[NUM_BOARDS][12],
             bool fullDischarge[NUM_BOARDS][12],
             uint8_t balanceCounter,
             uint8_t *chargeRate)
{
    (void)fullDischarge;
    (void)chargeRate;

    clear_discharge_matrix(cellDischarge);

    if (cfg == NULL || bms == NULL) {
        return;
    }

    if (bms->curr_max_voltage <= cfg->balancing_start_threshold) {
        return;
    }

    if (cfg->numOfCellsPerIC == 0U) {
        return;
    }

    const uint8_t rr_slot = (uint8_t)(balanceCounter % cfg->numOfCellsPerIC);

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        const uint16_t cellVoltage = bmsData[cell].voltage;
        const bool valid_voltage =
            (cellVoltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cellVoltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            continue;
        }

        if ((cellVoltage >= (uint16_t)(cfg->balancing_start_threshold + cfg->balancing_difference)) &&
            ((cell % cfg->numOfCellsPerIC) == rr_slot)) {
            const uint8_t boardIndex = (uint8_t)(cell / cfg->numOfCellsPerIC);
            const uint8_t cellIndex  = (uint8_t)(cell % cfg->numOfCellsPerIC);
            cellDischarge[boardIndex][cellIndex] = true;
        }
    }
}

void thresholdBalance(BMSConfigStructTypedef *cfg,
                      BMS_critical_info_t *bms,
                      CellData bmsData[NUM_CELLS],
                      bool cell_discharge[NUM_BOARDS][12],
                      uint16_t cell_discharge_threshold,
                      uint8_t num_cells_discharge_per_secondary)
{
    clear_discharge_matrix(cell_discharge);

    if (cfg == NULL || bms == NULL) {
        return;
    }

    if (cfg->numOfCellsPerIC == 0U || num_cells_discharge_per_secondary == 0U) {
        return;
    }

    if (bms->curr_max_voltage <= cell_discharge_threshold) {
        return;
    }

    uint8_t discharge_cell_modulo =
        (uint8_t)(cfg->numOfCellsPerIC / num_cells_discharge_per_secondary);

    if (discharge_cell_modulo == 0U) {
        discharge_cell_modulo = 1U;
    }

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        const uint16_t cell_voltage = bmsData[cell].voltage;
        const bool valid_voltage =
            (cell_voltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cell_voltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            continue;
        }

        if (cell_voltage > cell_discharge_threshold) {
            const uint8_t boardIndex = (uint8_t)(cell / cfg->numOfCellsPerIC);
            const uint8_t localCell   = (uint8_t)(cell % cfg->numOfCellsPerIC);

            if ((localCell % discharge_cell_modulo) == s_threshold_balance_counter) {
                cell_discharge[boardIndex][localCell] = true;
            }
        }
    }

    s_threshold_balance_counter++;

    if (s_threshold_balance_counter >= discharge_cell_modulo) {
        s_threshold_balance_counter = 0U;
    }
}

bool packImbalanceFault(const BMSConfigStructTypedef *cfg, const BMS_critical_info_t *bms)
{
    if (cfg == NULL || bms == NULL) {
        return false;
    }

    if (bms->curr_max_voltage < bms->curr_min_voltage) {
        return false;
    }

    return ((bms->curr_max_voltage - bms->curr_min_voltage) > cfg->max_difference);
}