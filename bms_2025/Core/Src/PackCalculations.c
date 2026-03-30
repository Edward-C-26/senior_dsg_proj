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
            // Clear every discharge request so each balancing pass starts from a blank slate.
            discharge[board][cell] = false;
        }
    }
}

void setCriticalVoltages(BMS_critical_info_t *bms, CellData const bmsData[NUM_CELLS])
{
    // Accumulate the total pack voltage in mV before storing it back into the status struct.
    uint32_t total_pack_mV = 0U;
    // Track whether at least one cell reading passed the validity check.
    bool valid_found = false;

    // Start with sentinel values so the first valid cell will replace them.
    bms->curr_min_voltage = UINT16_MAX;
    bms->curr_max_voltage = 0U;
    bms->invalid_data = false;
    bms->invalid_data_cell = 255U;
    bms->min_volt_cell = 255U;
    bms->max_volt_cell = 255U;

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        // Pull out the current cell voltage once so the comparisons stay readable.
        const uint16_t cellVoltage = bmsData[cell].voltage;
        // Treat obviously bad readings as invalid so they do not affect pack calculations.
        const bool valid_voltage =
            (cellVoltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cellVoltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            // Remember that at least one bad cell reading was seen and record where it happened.
            bms->invalid_data = true;
            bms->invalid_data_cell = cell;
            continue;
        }

        // Mark that we found at least one usable voltage reading.
        valid_found = true;

        /*
         * LTC6811 cell codes are typically in 100 uV/count.
         * 42000 -> 4.2000 V -> 4200 mV
         */
        // Convert the raw LTC6811 code to mV before adding it into the pack total.
        total_pack_mV += (uint32_t)(cellVoltage / PACK_TOTAL_VOLTAGE_DIVISOR_TO_MV);

        if (cellVoltage > bms->curr_max_voltage) {
            // Update the running pack maximum and remember which cell produced it.
            bms->curr_max_voltage = cellVoltage;
            bms->max_volt_cell = cell;
        }

        if (cellVoltage < bms->curr_min_voltage) {
            // Update the running pack minimum and remember which cell produced it.
            bms->curr_min_voltage = cellVoltage;
            bms->min_volt_cell = cell;
        }
    }

    if (!valid_found) {
        // Fall back to zeros when every reading was invalid so callers do not use stale extrema.
        bms->curr_min_voltage = 0U;
        bms->curr_max_voltage = 0U;
        bms->min_volt_cell = 255U;
        bms->max_volt_cell = 255U;
        bms->cellMonitorPackVoltage = 0U;
        return;
    }

    if (total_pack_mV > UINT16_MAX) {
        // Clamp the reported pack voltage if the total no longer fits in 16 bits.
        bms->cellMonitorPackVoltage = UINT16_MAX;
    } else {
        // Otherwise store the exact summed pack voltage in mV.
        bms->cellMonitorPackVoltage = (uint16_t)total_pack_mV;
    }
}

void setCriticalTemps(BMS_critical_info_t *bms, CellData const bmsData[NUM_CELLS])
{
    // Track whether we saw at least one temperature value while scanning the pack.
    bool valid_found = false;

    // Start with sentinel values so the first temperature reading replaces them.
    bms->curr_min_temp = UINT16_MAX;
    bms->curr_max_temp = 0U;
    bms->min_temp_cell = 255U;
    bms->max_temp_cell = 255U;

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        // Read the current cell temperature once so the comparisons stay simple.
        const uint16_t cellTemp = bmsData[cell].temperature;

        /* Do not treat 0C as automatically invalid */
        // Count every present reading as valid, including an actual 0 C value.
        valid_found = true;

        if (cellTemp > bms->curr_max_temp) {
            // Update the hottest cell seen so far.
            bms->curr_max_temp = cellTemp;
            bms->max_temp_cell = cell;
        }

        if (cellTemp < bms->curr_min_temp) {
            // Update the coolest cell seen so far.
            bms->curr_min_temp = cellTemp;
            bms->min_temp_cell = cell;
        }
    }

    if (!valid_found) {
        // Reset the extrema when no usable temperature data was available.
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
    // These inputs are unused in the current balancing strategy.
    (void)fullDischarge;
    (void)chargeRate;

    // Clear any old balancing requests before calculating the new ones.
    clear_discharge_matrix(cellDischarge);

    if (cfg == NULL || bms == NULL) {
        // Bail out early if the required config or status pointers are missing.
        return;
    }

    if (bms->curr_max_voltage <= cfg->balancing_start_threshold) {
        // Do not start balancing until the pack has crossed the configured threshold.
        return;
    }

    if (cfg->numOfCellsPerIC == 0U) {
        // Avoid divide-by-zero math if the board configuration is not initialized.
        return;
    }

    // Rotate which local cell slot is allowed to discharge on this balancing pass.
    const uint8_t rr_slot = (uint8_t)(balanceCounter % cfg->numOfCellsPerIC);

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        // Pull out the current cell voltage once for the validity checks below.
        const uint16_t cellVoltage = bmsData[cell].voltage;
        // Ignore readings that are clearly outside the believable voltage range.
        const bool valid_voltage =
            (cellVoltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cellVoltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            // Skip invalid cells so they never get selected for balancing.
            continue;
        }

        if ((cellVoltage >= (uint16_t)(cfg->balancing_start_threshold + cfg->balancing_difference)) &&
            ((cell % cfg->numOfCellsPerIC) == rr_slot)) {
            // Convert the flat cell index into its board number and local cell position.
            const uint8_t boardIndex = (uint8_t)(cell / cfg->numOfCellsPerIC);
            const uint8_t cellIndex  = (uint8_t)(cell % cfg->numOfCellsPerIC);
            // Request discharge only for cells above threshold that match this round-robin slot.
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
    // Clear any old threshold-balancing requests before evaluating the latest pack data.
    clear_discharge_matrix(cell_discharge);

    if (cfg == NULL || bms == NULL) {
        // Stop immediately if the caller did not provide valid pointers.
        return;
    }

    if (cfg->numOfCellsPerIC == 0U || num_cells_discharge_per_secondary == 0U) {
        // Guard against divide-by-zero when the hardware or balancing settings are invalid.
        return;
    }

    if (bms->curr_max_voltage <= cell_discharge_threshold) {
        // Skip balancing until the pack exceeds the caller's discharge threshold.
        return;
    }

    // Spread the allowed discharge cells across the secondary by stepping through a modulo pattern.
    uint8_t discharge_cell_modulo =
        (uint8_t)(cfg->numOfCellsPerIC / num_cells_discharge_per_secondary);

    if (discharge_cell_modulo == 0U) {
        // Fall back to 1 so the modulo logic still works when the division rounds to zero.
        discharge_cell_modulo = 1U;
    }

    for (uint8_t cell = 0U; cell < NUM_CELLS; cell++) {
        // Pull out the current voltage once so the threshold logic is easier to follow.
        const uint16_t cell_voltage = bmsData[cell].voltage;
        // Ignore any reading that falls outside the believable operating range.
        const bool valid_voltage =
            (cell_voltage > INVALID_VOLTAGE_LOWER_THRESHOLD) &&
            (cell_voltage < INVALID_VOLTAGE_UPPER_THRESHOLD);

        if (!valid_voltage) {
            // Invalid readings do not participate in balancing decisions.
            continue;
        }

        if (cell_voltage > cell_discharge_threshold) {
            // Convert the flat cell index into its board and local cell positions.
            const uint8_t boardIndex = (uint8_t)(cell / cfg->numOfCellsPerIC);
            const uint8_t localCell   = (uint8_t)(cell % cfg->numOfCellsPerIC);

            if ((localCell % discharge_cell_modulo) == s_threshold_balance_counter) {
                // Enable discharge only for the local cells assigned to this rotating modulo slot.
                cell_discharge[boardIndex][localCell] = true;
            }
        }
    }

    // Advance the rotating slot so a different subset of cells is chosen next time.
    s_threshold_balance_counter++;

    if (s_threshold_balance_counter >= discharge_cell_modulo) {
        // Wrap the slot counter back around once it reaches the modulo limit.
        s_threshold_balance_counter = 0U;
    }
}

bool packImbalanceFault(const BMSConfigStructTypedef *cfg, const BMS_critical_info_t *bms)
{
    if (cfg == NULL || bms == NULL) {
        // Without config or pack stats, there is not enough information to declare an imbalance.
        return false;
    }

    if (bms->curr_max_voltage < bms->curr_min_voltage) {
        // Reject obviously inconsistent extrema instead of producing a bogus fault.
        return false;
    }

    // Report a fault when the pack spread exceeds the configured maximum difference.
    return ((bms->curr_max_voltage - bms->curr_min_voltage) > cfg->max_difference);
}
