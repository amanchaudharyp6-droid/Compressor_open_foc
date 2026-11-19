#ifndef __VDQ_CIRCLE_LIMIT_H
#define __VDQ_CIRCLE_LIMIT_H

/* Circular |Vd,Vq| limit, d-axis priority. See FOC_VDQ_CIRCLE_LIMIT in
 * vehicle_config.h. Header-only and HAL-free so tools/host_tests can build it.
 *
 * A pair inside radius r is left untouched. A pair outside it is shortened:
 * Vd is kept, clipped to +/-vd_max (<= r), and Vq gets whatever is left,
 * sqrt(r^2 - Vd^2), with its sign kept. Field weakening holds the flux down
 * through Id, so the d axis keeps its voltage and the torque axis gives way;
 * vd_max < r guarantees Vq is never squeezed to zero, which at speed would be
 * a hard brake. Returns 1 if the vector was shortened, 0 if it was inside. */

#include <math.h>
#include <stdint.h>

static inline uint8_t Vdq_Circle_Limit(float *vd, float *vq, float r, float vd_max)
{
  if ((*vd * *vd) + (*vq * *vq) <= r * r) return 0;

  if (*vd > vd_max) *vd = vd_max;
  else if (*vd < -vd_max) *vd = -vd_max;

  float vq_max = sqrtf(fmaxf((r * r) - (*vd * *vd), 0.0f));
  if (*vq > vq_max) *vq = vq_max;
  else if (*vq < -vq_max) *vq = -vq_max;

  return 1;
}

#endif /* __VDQ_CIRCLE_LIMIT_H */
