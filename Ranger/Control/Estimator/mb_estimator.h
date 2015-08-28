#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

#include <RangerMath.h>   // Tan(), bool

extern bool INITIALIZE_ESTIMATOR; // Should the estimator be initialized?

bool getContactOuter(void);
bool getContactInner(void);

void mb_estimator_update(void);

#endif  // __MB_ESTIMATOR_H__

