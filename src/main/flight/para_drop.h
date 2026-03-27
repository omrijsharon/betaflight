#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

typedef struct paraDropConfig_s {
    uint16_t aslThresholdMeters;
    uint8_t servoChannel;
    uint16_t holdTimeMs;
} paraDropConfig_t;

PG_DECLARE(paraDropConfig_t, paraDropConfig);

void paraDropInit(void);
void paraDropReset(void);
void paraDropUpdate(timeUs_t currentTimeUs);
bool paraDropIsEnabled(void);
bool paraDropRequiresServoOutput(void);
