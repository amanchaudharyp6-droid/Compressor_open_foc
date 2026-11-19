/* Host test for Vdq_Circle_Limit (Core/Inc/Vdq_Circle_Limit.h).
 *
 * and Isense_Park_Angle (Core/Inc/Isense_Delay_Comp.h).
 *
 *   gcc -std=c11 -Wall -Wextra -I Core/Inc tools/host_tests/test_vdq_circle_limit.c -lm -o test_vdq && ./test_vdq
 *
 * Run from the repo root. Exit code 0 = all pass. */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Vdq_Circle_Limit.h"
#include "Isense_Delay_Comp.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define NEAR(a, b) (fabsf((a) - (b)) <= 1e-4f * (1.0f + fabsf(b)))

static void expect(float vd, float vq, float r, float vd_max, float vd_exp, float vq_exp, uint8_t clip_exp)
{
  float d_ = vd, q_ = vq;
  uint8_t clip = Vdq_Circle_Limit(&d_, &q_, r, vd_max);
  CHECK(NEAR(d_, vd_exp) && NEAR(q_, vq_exp) && clip == clip_exp,
        "(%g, %g) r=%g -> (%g, %g) clip=%u, want (%g, %g) clip=%u",
        vd, vq, r, d_, q_, clip, vd_exp, vq_exp, clip_exp);
}

/* Mirrors the current-PI integrator update in FOC_Loop.c for one axis held
 * at a constant error with the regulator out of voltage (the windup case).
 * circle = 0: today's back-calculation, which only sees the PI's own clamp.
 * circle = 1: back-calculation against the final limited voltage.
 * Returns the integrator state after `seconds`. */
static float windup(int circle, float seconds, float *v_final_out)
{
  const float ts = 1.0e-4f, kp = 0.15f, ki = 4.0f, kaw = 4.8304589f;  /* TR q-axis */
  const float r = 48.0f / 1.7320508f;
  const float vd = -20.0f;     /* d-axis demand, kept by d priority */
  const float vq_ff = 20.0f;   /* back-EMF feed-forward */
  const float err = 10.0f;     /* A, sustained: the voltage isn't there to close it */
  float integ = 0.0f, v_final = 0.0f;

  for (int k = 0; k < (int)(seconds / ts); k++) {
    float pi_unsat = kp * err + integ;
    float pi_sat = fmaxf(-r, fminf(r, pi_unsat));
    float vd_l = vd, vq_l = fmaxf(-r, fminf(r, pi_sat + vq_ff));  /* per-axis clamp */
    if (circle) (void)Vdq_Circle_Limit(&vd_l, &vq_l, r, 0.9f * r);
    v_final = vq_l;
    float pi_applied = circle ? (vq_l - vq_ff) : pi_sat;
    integ += ((pi_applied - pi_unsat) * kaw + err * ki) * ts;
    integ = fmaxf(-r, fminf(r, integ));
  }
  *v_final_out = v_final;
  return integ;
}

int main(void)
{
  const float r = 10.0f;

  /* inside or on the circle: untouched, even with |Vd| above vd_max */
  expect(3.0f, 4.0f, r, r, 3.0f, 4.0f, 0);
  expect(6.0f, 8.0f, r, r, 6.0f, 8.0f, 0);
  expect(-9.9f, 0.0f, r, r, -9.9f, 0.0f, 0);
  expect(-9.5f, 1.0f, r, 0.9f * r, -9.5f, 1.0f, 0);

  /* outside: Vd kept, Vq trimmed, sign of Vq kept */
  expect(-6.0f, 10.0f, r, r, -6.0f, 8.0f, 1);
  expect(-6.0f, -10.0f, r, r, -6.0f, -8.0f, 1);
  expect(6.0f, 9.0f, r, 0.9f * r, 6.0f, 8.0f, 1);

  /* Vd alone past the circle, vd_max = r: Vd on the rail, nothing left for Vq */
  expect(-12.0f, 5.0f, r, r, -10.0f, 0.0f, 1);
  expect(15.0f, 0.0f, r, r, 10.0f, 0.0f, 1);
  expect(0.0f, -12.0f, r, r, 0.0f, -10.0f, 1);

  /* vd_max = 0.9 r: Vq always keeps sqrt(1 - 0.81) = 0.436 r */
  expect(-12.0f, 5.0f, r, 9.0f, -9.0f, 4.3589f, 1);
  expect(-9.5f, -8.0f, r, 9.0f, -9.0f, -4.3589f, 1);
  expect(-12.0f, 2.0f, r, 9.0f, -9.0f, 2.0f, 1);   /* Vq already inside its room */

  /* sweep: never outside the circle, |Vd| <= vd_max when clipped, untouched when inside */
  srand(1);
  for (int i = 0; i < 100000; i++) {
    float vd = ((float)rand() / RAND_MAX - 0.5f) * 4.0f * r;
    float vq = ((float)rand() / RAND_MAX - 0.5f) * 4.0f * r;
    float vd_max = (i & 1) ? r : 0.9f * r;
    float d_ = vd, q_ = vq;
    uint8_t clip = Vdq_Circle_Limit(&d_, &q_, r, vd_max);
    float mag = sqrtf(d_ * d_ + q_ * q_);
    int inside = (vd * vd + vq * vq) <= r * r;
    CHECK(mag <= r * (1.0f + 1e-5f), "(%g, %g) -> |%g| > r", vd, vq, mag);
    CHECK(inside ? (clip == 0 && d_ == vd && q_ == vq) : clip == 1,
          "(%g, %g) inside=%d clip=%u -> (%g, %g)", vd, vq, inside, clip, d_, q_);
    if (!inside) {
      CHECK(NEAR(d_, fmaxf(-vd_max, fminf(vd_max, vd))), "(%g, %g) -> Vd %g not clamp(Vd, vd_max)", vd, vq, d_);
      CHECK(fabsf(q_) >= fminf(fabsf(vq), sqrtf(r * r - vd_max * vd_max)) * (1.0f - 1e-5f),
            "(%g, %g) -> Vq %g lost more than the circle needs", vd, vq, q_);
    }
    CHECK(q_ * vq >= 0.0f, "(%g, %g) -> Vq sign flipped to %g", vd, vq, q_);
  }

  /* current-sense delay: the ADC hands over currents from tau ago. Park with
   * the sampling angle rotates the measured vector by -w*tau (the 28_10 log:
   * a mostly -d vector reads as large +Iq). Park with Isense_Park_Angle()
   * must give the true dq back, for either rotation direction. */
  {
    const float tau = 180.0e-6f, id_true = -70.0f, iq_true = 5.0f;
    const float w_list[2] = {4189.0f, -4189.0f};   /* +/-8000 rpm electrical, 5 pp */
    for (int k = 0; k < 2; k++) {
      float w = w_list[k];
      for (float t = 0.0f; t < 2.0e-3f; t += 1.0e-4f) {
        float th_now = w * t, th_smp = w * (t - tau);
        float a = id_true * cosf(th_smp) - iq_true * sinf(th_smp);   /* inverse Park */
        float b = id_true * sinf(th_smp) + iq_true * cosf(th_smp);
        float th = Isense_Park_Angle(th_now, w, tau);
        float id_m = a * cosf(th) + b * sinf(th);                     /* FOC_Loop.c Park */
        float iq_m = b * cosf(th) - a * sinf(th);
        CHECK(fabsf(id_m - id_true) < 1e-3f && fabsf(iq_m - iq_true) < 1e-3f,
              "w=%g t=%g: compensated (%g, %g), want (%g, %g)", w, t, id_m, iq_m, id_true, iq_true);
        float id_u = a * cosf(th_now) + b * sinf(th_now);
        float iq_u = b * cosf(th_now) - a * sinf(th_now);
        if (w > 0.0f) {
          CHECK(iq_u > iq_true + 30.0f && id_u > id_true + 10.0f,
                "w=%g: uncompensated (%g, %g) should show the logged signature", w, id_u, iq_u);
        }
      }
    }
  }

  /* anti-windup: today's integrator rides to its rail; with the back-calc on
   * the final voltage it settles where Kaw*(excess) balances Ki*err */
  {
    float v_old, v_new;
    float i_old = windup(0, 1.0f, &v_old);
    float i_new = windup(1, 1.0f, &v_new);
    const float r_tr = 48.0f / 1.7320508f;
    float vq_room = sqrtf(r_tr * r_tr - 20.0f * 20.0f);
    float i_eq = (vq_room - 20.0f) - 0.15f * 10.0f + 4.0f * 10.0f / 4.8304589f;
    CHECK(NEAR(i_old, r_tr), "old integrator %g, expected to wind to rail %g", i_old, r_tr);
    CHECK(NEAR(v_new, vq_room), "final Vq %g, want circle room %g", v_new, vq_room);
    CHECK(fabsf(i_new - i_eq) < 0.05f, "new integrator %g, want equilibrium %g", i_new, i_eq);
    printf("windup after 1 s at 10 A error: old integ %.2f V (rail), new %.2f V (eq %.2f)\n",
           i_old, i_new, i_eq);
  }

  printf(failures ? "%d FAILED\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
