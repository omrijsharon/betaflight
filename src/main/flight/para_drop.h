#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

typedef struct paraDropConfig_s {
    int16_t altitudeMeters;
    uint8_t servoChannel;
    uint8_t holdTimeSec;
} paraDropConfig_t;

PG_DECLARE(paraDropConfig_t, paraDropConfig);

void paraDropInit(void);
void paraDropReset(void);
void paraDropUpdate(timeUs_t currentTimeUs);
bool paraDropIsEnabled(void);
bool paraDropRequiresServoOutput(void);
