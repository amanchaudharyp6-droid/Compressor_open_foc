#ifndef __CAN_RX_HANDLERS_H_
#define __CAN_RX_HANDLERS_H_

#include "main.h"

/* Dispatch a received CAN frame to the appropriate per-ID handler.
 * Call this inside HAL_FDCAN_RxFifo0Callback after verifying DataLength.
 * Returns without action if no handler is registered for the given ID. */
void can_rx_dispatch(uint32_t id, const uint8_t *msg);
void can_rx_0x729(const uint8_t *msg);

#endif /* __CAN_RX_HANDLERS_H_ */
