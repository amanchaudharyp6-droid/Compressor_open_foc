#ifndef __MOTOR_SELECT_H
#define __MOTOR_SELECT_H

#define MOTOR_PROFILE_DEFAULT		0
#define MOTOR_PROFILE_LIVGUARD		1
#define MOTOR_PROFILE_HIRANGE_TR	2

/* Override at the build system level (e.g. -DMOTOR_PROFILE=MOTOR_PROFILE_LIVGUARD)
 * or change the default below. */
#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE	MOTOR_PROFILE_HIRANGE_TR
#endif

#if MOTOR_PROFILE == MOTOR_PROFILE_DEFAULT
	#include "motor_default.h"
#elif MOTOR_PROFILE == MOTOR_PROFILE_LIVGUARD
	#include "motor_livguard.h"
#elif MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR
	#include "motor_hirange_tr.h"
#else
	#error "Unknown MOTOR_PROFILE selected — see motor_select.h"
#endif

#endif /* __MOTOR_SELECT_H */
