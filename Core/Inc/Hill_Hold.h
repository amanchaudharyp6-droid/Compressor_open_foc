#ifndef __HILL_HOLD_H_
#define __HILL_HOLD_H_

#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * HILL HOLD — speed PI layered on top of the torque-mode FOC path.
 *
 * Engages when the vehicle rolls OPPOSITE to the engaged drive direction
 * faster than HH_ENTRY_RPM with the brake released, and holds it at 0 RPM
 * by raising the torque reference. Applied as a floor on RefTrq
 * (max(throttle, hill_hold)) so the driver's throttle takes over seamlessly.
 *
 * Call HillHold_Apply() from isr_foc_start() immediately before rt_OneStep();
 * it decimates internally from the 10 kHz ISR rate to a 1 kHz PI update.
 * All state lives in ISR context — do not call from thread/main context.
 *
 * Tuning knobs live in vehicle_config.h (HH_* macros, HILL_HOLD_ENABLE).
 * ========================================================================= */

/* Run the hold state machine and return the (possibly raised) torque
 * reference in Nm.
 *
 * Pass d.torque_final — the superloop's driver-demand variable, which this
 * module never writes. Do NOT pass FOC_Loop_U.RefTrq: that is where the
 * caller stores this function's result, so the module would read its own
 * output back as "driver demand" between superloop writes. The max() floor
 * then becomes a torque ratchet, and the driver-takeover release compares
 * the hold torque against itself. */
float HillHold_Apply(float ref_torque_nm);

/* True while the hold is producing torque (holding or ramping out). */
bool HillHold_IsActive(void);

/* Last hill-hold torque output in Nm (0 when inactive) — for CAN/debug. */
float HillHold_GetTorque(void);

/* Raw state-machine code — for CAN/debug logging:
 *   0 = INACTIVE (armed, waiting), 1 = ACTIVE (holding),
 *   2 = RELEASING (budget spent, ramping out), 3 = COOLDOWN,
 *   4 = STANDBY (driver took over; grade estimate retained for re-entry). */
uint8_t HillHold_GetState(void);

#endif /* __HILL_HOLD_H_ */
