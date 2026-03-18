# Camera Lock MSP Guide

## Purpose

This file explains the three custom MSP messages used for camera-lock data in this Betaflight fork:

- `MSP_CAMERA_INFO` = `190`
- `MSP_CAMERA_GET_LOCK` = `191`
- `MSP_CAMERA_LOCK` = `192`

The goal is to let another agent understand:

- what each message means
- what the payload layout is
- what the fields represent
- how firmware code and external tools should use them

These commands are private fork extensions and are intended to be compiled under `USE_FINAL`.

## General Notes

- MSP version: these commands are used as custom MSP v1 IDs
- payload byte order: little-endian
- lock coordinates are image pixel coordinates
- image origin is top-left
- all camera-lock state in the FC is owned by `src/main/sensors/camera_lock.c`

There are two layers of lock data:

1. **raw lock state**
   - latest lock information received from the upstream camera/vision source
2. **FC-derived lock state**
   - same lock position plus FC-side `age_ms` and `fresh` calculation

That is why there are separate `GET_LOCK` and `LOCK` messages.

## Message 190: `MSP_CAMERA_INFO`

### Meaning

Returns the active camera model/configuration information needed to interpret lock coordinates.

This is static or slow-changing metadata:

- active image size
- camera intrinsics
- FOV
- camera tilt
- camera orientation
- nominal lock/detection rate
- info flags

### Payload

```c
uint16 width_px;
uint16 height_px;
uint32 fx_px_x1000;
uint32 fy_px_x1000;
uint32 cx_px_x1000;
uint32 cy_px_x1000;
uint8  hfov_deg;
uint8  vfov_deg;
int8   tilt_angle_deg;
uint8  orientation;
uint16 lock_rate_hz;
uint8  flags;
```

### Field meanings

- `width_px`
  - active image width in pixels
- `height_px`
  - active image height in pixels
- `fx_px_x1000`
  - focal length in x, in pixels times `1000`
- `fy_px_x1000`
  - focal length in y, in pixels times `1000`
- `cx_px_x1000`
  - principal point x, in pixels times `1000`
- `cy_px_x1000`
  - principal point y, in pixels times `1000`
- `hfov_deg`
  - horizontal field of view, integer degrees
- `vfov_deg`
  - vertical field of view, integer degrees
- `tilt_angle_deg`
  - camera tilt around the pitch axis, signed degrees, valid range `-90 .. +90`
- `orientation`
  - rotation around the camera front axis:
    - `0` = `0 deg`
    - `1` = `+90 deg`
    - `2` = `180 deg`
    - `3` = `-90 deg`
- `lock_rate_hz`
  - nominal upstream lock/detection rate
- `flags`
  - info validity/health bits

### Info flags

Defined in `camera_lock.h`:

- bit0 = `CAMERA_LOCK_INFO_FLAG_VALID`
- bit1 = `CAMERA_LOCK_INFO_FLAG_CROPPED`
- bit2 = `CAMERA_LOCK_INFO_FLAG_INTRINSICS_VALID`
- bit3 = `CAMERA_LOCK_INFO_FLAG_FOV_VALID`
- bit4 = `CAMERA_LOCK_INFO_FLAG_HEALTHY`

### How another agent should use it

Use this message when code needs to:

- map image coordinates to OSD coordinates
- understand the active image size
- decode intrinsics
- know whether the camera info is valid and healthy

For intrinsics:

- divide `*_x1000` values by `1000.0`
- the FC already exposes helper decode functions in `camera_lock.c`

## Message 191: `MSP_CAMERA_GET_LOCK`

### Meaning

Returns the latest **raw** lock state stored in the FC.

This is the closest representation of what the upstream camera/vision source most recently provided.

It does **not** include FC-derived age or freshness.

### Payload

```c
uint8  flags;
uint16 x_px;
uint16 y_px;
```

### Field meanings

- `flags`
  - raw lock flags
- `x_px`
  - raw lock x coordinate in active image pixels
- `y_px`
  - raw lock y coordinate in active image pixels

### Raw lock flags

Defined in `camera_lock.h`:

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`

Note:

- `FRESH` is **not** part of the raw state
- freshness is computed by the FC from local time

### How another agent should use it

Use this message when you want:

- the last raw lock position
- the raw upstream lock bits
- a direct view of the cached source data without FC freshness logic

This is useful for:

- debugging the upstream lock producer
- checking whether the FC is caching incoming lock coordinates correctly

## Message 192: `MSP_CAMERA_LOCK`

### Meaning

Returns the FC-derived lock state.

This is the message that other FC features should normally trust for rendering or behavior decisions, because it includes:

- `age_ms`
- `fresh`

### Payload

```c
uint8  flags;
uint16 x_px;
uint16 y_px;
uint16 age_ms;
```

### Field meanings

- `flags`
  - FC-derived lock flags
- `x_px`
  - lock x coordinate in active image pixels
- `y_px`
  - lock y coordinate in active image pixels
- `age_ms`
  - age of the cached lock sample, computed by the FC from its own clock

### FC-derived lock flags

Defined in `camera_lock.h`:

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`
- bit2 = `CAMERA_LOCK_FLAG_FRESH`

### Freshness behavior

The FC computes freshness using:

- last camera-lock update time
- current FC time in microseconds
- `CAMERA_LOCK_DEFAULT_FRESHNESS_THRESHOLD_MS`

Current default:

- `100 ms`

### How another agent should use it

Use this message for:

- OSD lock rendering
- behavior that must ignore stale data
- anything that should depend on current valid lock state

This is the preferred message for consumers that want:

- `detected`
- `healthy`
- `fresh`
- current lock position

## Expected usage pattern

### FC internal usage

- `camera_lock.c` stores camera info and raw lock state
- `msp.c` exposes that state over MSP
- OSD lock overlay uses the FC-derived state from `camera_lock.c`, not raw placeholders

### External client usage

Recommended order:

1. request `MSP_CAMERA_INFO`
2. request `MSP_CAMERA_LOCK`
3. only use the lock for display/control if:
   - `DETECTED` is set
   - `HEALTHY` is set
   - `FRESH` is set

Use `MSP_CAMERA_GET_LOCK` only if you specifically want raw cached values.

## Wire sizes

### `MSP_CAMERA_INFO`

Payload size:

- `2 + 2 + 4 + 4 + 4 + 4 + 1 + 1 + 1 + 1 + 2 + 1 = 27 bytes`

MSP v1 response frame size:

- `27 + 6 = 33 bytes`

### `MSP_CAMERA_GET_LOCK`

Payload size:

- `1 + 2 + 2 = 5 bytes`

MSP v1 response frame size:

- `11 bytes`

### `MSP_CAMERA_LOCK`

Payload size:

- `1 + 2 + 2 + 2 = 7 bytes`

MSP v1 response frame size:

- `13 bytes`

## Important implementation notes

- `MSP_CAMERA_INFO` is meaningful only if `CAMERA_LOCK_INFO_FLAG_VALID` is set
- if `width_px <= 1` or `height_px <= 1`, lock-to-OSD mapping should be treated as invalid
- `tilt_angle_deg` is signed
- `orientation` and `tilt_angle_deg` are different concepts:
  - `orientation` = image rotation
  - `tilt_angle_deg` = camera pitch/tilt angle
- intrinsics are fixed-point `x1000`, not integers in plain pixels

## Current FC ownership

The current source of truth is:

- [camera_lock.h](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.h)
- [camera_lock.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.c)

The MSP serialization lives in:

- [msp.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/msp/msp.c)

Another agent extending this feature should update the `camera_lock` module first, then let MSP and OSD read from it.
