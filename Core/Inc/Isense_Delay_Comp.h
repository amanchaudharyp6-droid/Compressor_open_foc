#ifndef __ISENSE_DELAY_COMP_H
#define __ISENSE_DELAY_COMP_H

/* Current-sense delay compensation. See FOC_ISENSE_DELAY_COMP in
 * vehicle_config.h. Header-only and HAL-free so tools/host_tests can build it.
 *
 * The phase currents handed to the Park transform are tau_s old, so the rotor
 * angle that matches them is the present angle minus w_e * tau_s. omega_e is
 * signed (d.omega_e), which keeps this right for either rotation direction. */

static inline float Isense_Park_Angle(float theta, float omega_e, float tau_s)
{
  return theta - omega_e * tau_s;
}

#endif /* __ISENSE_DELAY_COMP_H */
