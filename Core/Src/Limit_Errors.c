#include "Limit_Errors.h"

uint8_t Over_Current_Phase_Error(float a, float b, float c) {
    volatile float max_curr = fmaxf(a, fmaxf(b, c));
    volatile float min_curr = fminf(a, fminf(b, c));

    static uint16_t error_counter = 0;

    if (max_curr > MAX_PHASE_CURRENT || min_curr < -MAX_PHASE_CURRENT) {
        error_counter++;

        if (error_counter >= PHASE_OC_COUNT) {
            error_counter = 0;
            er.error_c2 |= (1 << 3);
            return !HAL_OK;
        }
    } else {
        if (error_counter <= 0) {
            error_counter = 0;
        } else {
            error_counter--;
        }
    }

    return HAL_OK;
}

uint8_t Over_Current_Id_Iq_Error(float Id, float Iq) {
    static uint16_t error_counter = 0;

    if (Id > Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit || Id < Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit || Iq < Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit || Iq > Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit) {
        error_counter++;

        if (error_counter >= ID_IQ_OC_COUNT) {
            error_counter = 0;
            er.error_c2 |= (1 << 4);
            return !HAL_OK;
        }
    } else {
        if (error_counter <= 0) {
            error_counter = 0;
        } else {
            error_counter--;
        }
    }

    return HAL_OK;
}

uint8_t Stall_Protection(float rpm, float irms) {
    static uint16_t error_counter = 0;

    if (fabsf(rpm) <= 100.0f && irms >= 150.0f) {
        error_counter++;

        if (error_counter >= STALL_COUNT) {
            error_counter = 0;
            er.error_c1 |= (1 << 2);
            return !HAL_OK;
        }
    } else {
        if (error_counter <= 0) {
            error_counter = 0;
        } else {
            error_counter--;
        }
    }

    return HAL_OK;
}

/* Only profiles that define OVERSPEED_TRIP_RPM get the trip; the others
 * compile it out and behave as before. */
uint8_t Overspeed_Error(float rpm) {
#if defined(OVERSPEED_TRIP_RPM)
    static uint16_t error_counter = 0;

    if (fabsf(rpm) >= OVERSPEED_TRIP_RPM) {
        error_counter++;

        if (error_counter >= OVERSPEED_COUNT) {
            error_counter = 0;
            er.error_c1 |= (1 << 1);   /* overspeed = bit1, vehicle matrix */
            return !HAL_OK;
        }
    } else {
        if (error_counter <= 0) {
            error_counter = 0;
        } else {
            error_counter--;
        }
    }
#else
    (void)rpm;
#endif

    return HAL_OK;
}

uint8_t Bus_Voltage_Error(float Bus_DC) {
    static uint16_t error_counter_ov = 0;
    static uint16_t error_counter_uv = 0;

    if (Bus_DC >= BUS_DC_OV_LIMIT) {
        error_counter_ov++;

        if (error_counter_ov >= BUS_DC_OV_COUNT) {
            error_counter_ov = 0;
            er.error_c1 |= (1 << 4);   /* OV = bit4, matches production error_code */
            return (uint8_t)1;
        }
    } else {
        if (error_counter_ov <= 0) {
            error_counter_ov = 0;
        } else {
            error_counter_ov--;
        }
    }

    if (Bus_DC <= BUS_DC_UV_LIMIT) {
        error_counter_uv++;

        if (error_counter_uv >= BUS_DC_UV_COUNT) {
            error_counter_uv = 0;
            er.error_c1 |= (1 << 5);   /* UV = bit5, matches production error_code */
            return (uint8_t)2;
        }
    } else {
        if (error_counter_uv <= 0) {
            error_counter_uv = 0;
        } else {
            error_counter_uv--;
        }
    }

    return HAL_OK;
}

uint8_t Aux_Voltage_Error(float Aux_DC) {
    /* Checking for Aux DC UV Error explicitly */
    if (Aux_DC <= AUX_UV_LIMIT && d.motor_start == 1) {
        er.error_c3 |= (1 << 1);
        return !HAL_OK;
    } else {
        return HAL_OK;
    }
}

uint8_t Mtr_OT_Error(float motor_temp) {
    if (motor_temp >= MOTOR_TEMP_OT_LIMIT) {
        er.error_c1 |= (1 << 3);
        return !HAL_OK;
    } else {
        return HAL_OK;
    }
}

uint8_t Mtc_OT_Error(float controller_temp) {
    if (controller_temp >= CONTRL_TEMP_OT_LIMIT) {
        return !HAL_OK;
    } else {
        return HAL_OK;
    }
}

void Error_Check(void) {
    if (Over_Current_Phase_Error(FOC_Loop_U.PhaseCurrent[0], FOC_Loop_U.PhaseCurrent[1], FOC_Loop_U.PhaseCurrent[2]) == !HAL_OK) {
        er.error_triggered = 1;
        er.phase_curr_error = 1;
        err = PHASE_CURR_ERROR;
        cS = CONT_ERROR;
    }

    if (Over_Current_Id_Iq_Error(FOC_Loop_Y.Id, FOC_Loop_Y.Iq) == !HAL_OK) {
        er.error_triggered = 1;
        er.id_iq_oc_error = 1;
        err = ID_IQ_OC_ERROR;
        cS = CONT_ERROR;
    }

    if (Stall_Protection(fabsf(d.RPM), d.irms) == !HAL_OK) {
        er.error_triggered = 1;
        er.stall_protection = 1;
        err = STALL_PROTECTION;
        cS = CONT_ERROR;
    }

    if (Overspeed_Error(d.RPM) == !HAL_OK) {
        er.error_triggered = 1;
        er.overspeed_error = 1;
        err = OVERSPEED_ERROR;
        cS = CONT_ERROR;
    }

    if (Bus_Voltage_Error(d.Vdc) == 1) {
        er.error_triggered = 1;
        er.bus_voltage_ov_error = 1;
        err = BUS_VOLTAGE_OV_ERROR;
        cS = CONT_ERROR;
    }

    if (Bus_Voltage_Error(d.Vdc) == 2) {
        er.error_triggered = 1;
        er.bus_voltage_uv_error = 1;
        err = BUS_VOLTAGE_UV_ERROR;
        cS = CONT_ERROR;
    }

    /* 12 V aux sense and motor NTC removed from this board: no aux-UV or
     * motor-OT checks (Aux_Voltage_Error / Mtr_OT_Error kept, just not called). */

    if (Mtc_OT_Error(d.Mtc_temp) == !HAL_OK) {
        er.error_triggered = 1;
        er.mtc_temp_ot_error = 1;
        err = MTC_TEMP_OT_ERROR;
        cS = CONT_ERROR;
    }

    disable_drive();
}