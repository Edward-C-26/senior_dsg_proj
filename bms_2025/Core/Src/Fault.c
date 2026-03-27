#include "Fault.h"

/*
 * Initialize the BMS critical-info structure to a known startup state.
 *
 * All tracked extrema, fault flags, and associated cell indices are reset so
 * later processing can safely populate them with fresh pack data.
 */
void init_BMS_info(BMS_critical_info_t * bms_struct) {
    bms_struct->curr_max_voltage = 0;
    bms_struct->max_volt_cell = 0;
    bms_struct->curr_min_voltage = 0;
    bms_struct->min_volt_cell = 0;

    bms_struct->curr_max_temp = 0;
    bms_struct->max_temp_cell = 0;
    bms_struct->curr_min_temp = 0;
    bms_struct->min_temp_cell = 0;

    bms_struct->invalid_data = false;
    bms_struct->invalid_data_cell = 255;

    bms_struct->cell_connection_fault = false;
    bms_struct->cell_connection_num = 255;

    bms_struct->is_fault = false;
    bms_struct->fault_board_num = 255;
}


/*
 * Evaluate the latest pack summary data against the configured protection
 * thresholds and encode the result into the outgoing BMS status bytes.
 *
 * Status byte 0 stores fault flags, while later bytes store the cell index
 * associated with the triggered condition when applicable.
 */
bool FAULT_check(BMS_critical_info_t *bms_struct, const BMSConfigStructTypedef *cfg, uint8_t bmsStatus[6]) {
    bool BMS_fault = false;
    memset(bmsStatus, 0, 6);

    if (bms_struct->curr_max_voltage > cfg->OV_threshold) {
        BMS_fault = true;
        bmsStatus[0] |= 0x01;
        bmsStatus[1] = bms_struct->max_volt_cell + 1;
    }

    if (bms_struct->curr_min_voltage < cfg->UV_threshold) {
        BMS_fault = true;
        bmsStatus[0] |= 0x02;
        bmsStatus[2] = bms_struct->min_volt_cell + 1;
    }

    if (bms_struct->curr_max_temp > cfg->OT_threshold) {
        BMS_fault = true;
        bmsStatus[0] |= 0x04;
        bmsStatus[3] = bms_struct->max_temp_cell + 1;
    }

    if (bms_struct->curr_min_temp < cfg->UT_threshold) {
        BMS_fault = true;
        bmsStatus[0] |= 0x08;
        bmsStatus[4] = bms_struct->min_temp_cell + 1;
    }

    /* Flag invalid ADC data reported by the LTC monitor path. */
    if (bms_struct->invalid_data) {
        BMS_fault = true;
        bmsStatus[0] |= 0x10;
        bmsStatus[5] = bms_struct->invalid_data_cell + 1;
    }

    bms_struct->is_fault = BMS_fault;
    return BMS_fault;
}
