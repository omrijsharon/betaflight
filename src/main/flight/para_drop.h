#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

#define PARA_DROP_ACTIVATION_KEY_LENGTH 16

typedef struct paraDropConfig_s {
    uint16_t aslThresholdMeters;
    uint8_t servoChannel;
    uint16_t holdTimeMs;
    uint8_t aboveHoldTimeSec;
    uint8_t indicatorPinio;
    uint8_t indicatorBlinkHz;
    char activationKey[PARA_DROP_ACTIVATION_KEY_LENGTH + 1];
} paraDropConfig_t;

PG_DECLARE(paraDropConfig_t, paraDropConfig);

void paraDropInit(void);
void paraDropReset(void);
void paraDropUpdate(timeUs_t currentTimeUs);
bool paraDropIsEnabled(void);
bool paraDropRequiresServoOutput(void);
