#ifndef __ISR_HANDLERS_H_
#define __ISR_HANDLERS_H_

#include "main.h"

/* ADC injected samples and calibration accumulators (owned by main.c) */
extern uint16_t injectedVal[3];
extern uint16_t currSensOff[3];
extern uint16_t counter;
extern uint32_t ia_offset;
extern uint32_t ib_offset;

/* Clamp Va/Vb/Vc to ±Vmax_SVM and write duty cycles to TIM1 */
void apply_SVM_voltages(void);

/* Per-state handlers called from HAL_ADCEx_InjectedConvCpltCallback */
void isr_curr_sens_calib(void);
void isr_angle_calib(void);
void isr_foc_start(void);
void isr_open_foc(void);

#endif /* __ISR_HANDLERS_H_ */
