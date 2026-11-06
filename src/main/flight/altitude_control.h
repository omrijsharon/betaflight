#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pg/pg.h"

#define ALT_CONTROL_CONFIG_VERSION 2
#define ALT_CONTROL_CONFIG_PAYLOAD_SIZE 19

// name, minimum, maximum, default. Append-only PG layout; order is the v2 MSP wire contract.
#define ALT_CONTROL_SETTINGS(X) \
    X(z_p, 0, 500, 100) \
    X(z_d, 0, 200, 40) \
    X(v_p, 0, 1200, 300) \
    X(v_i, 0, 1200, 100) \
    X(v_d, 0, 200, 0) \
    X(speed_cms, 10, 500, 100) \
    X(accel_cms2, 10, 300, 100) \
    X(jerk_cms3, 20, 1000, 400) \
    X(center_capture, 0, 200, 100)

typedef struct {
#define ALT_FIELD(n, lo, hi, d) uint16_t n;
    ALT_CONTROL_SETTINGS(ALT_FIELD)
#undef ALT_FIELD
} altitudeControlConfig_t;
PG_DECLARE(altitudeControlConfig_t, altitudeControlConfig);

typedef enum { AC_OFF, AC_WAIT_CENTER, AC_HOLD, AC_SPEED, AC_LOCKED } altitudeControlMode_e;
typedef enum { AC_OK, AC_SWITCH, AC_CONFLICT, AC_ESTIMATE, AC_ENTRY, AC_TIMEOUT, AC_WAIT_ANGLE } altitudeControlReason_e;

typedef struct {
    uint32_t timeUs;
    uint8_t mode, reason;
    int16_t stick;
    // SI units, positive up. Collective is normalized before motor mixing.
    float z, v, zRef, vRef, aRef, vCmd, p, i, d, output, ceiling, tilt;
} altitudeControlStatus_t;

const altitudeControlStatus_t *altitudeControlStatus(void);
bool altitudeControlActive(void);
float altitudeControlCollective(float outgoing);
void altitudeControlRelease(uint8_t reason);
void altitudeControlStep(uint32_t now, bool armed, bool requested, bool allowed, bool healthy,
    int stick, float z, float v, float tilt, float manualCollective, float ceiling);
