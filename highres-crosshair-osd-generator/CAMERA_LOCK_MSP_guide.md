# Camera Lock MSP Guide

## Purpose

This guide describes the current `USE_FINAL` camera-lock protocol in this Betaflight fork after the seeker/FPV split.

The design now has two layers:

- mandatory seeker lock ingestion
- optional FPV projection configuration

The seeker is always the live MSP peripheral that:

- registers itself to the FC
- gets bound to one MSP source/port
- gets polled by the FC for lock samples

FPV projection config is optional metadata stored in the FC. It is only required when the seeker declares that projecting onto an FPV camera/OSD view is mandatory.

## Command List

- `MSP_SEEKER_CAM_INFO` = `190`
- `MSP_CAMERA_GET_LOCK` = `191`
- `MSP_CAMERA_LOCK` = `192`
- `MSP_SET_SEEKER_CAM_INFO` = `193`
- `MSP_FPV_CAM_INFO` = `194`
- `MSP_SET_FPV_CAM_INFO` = `195`

## General Notes

- MSP version: custom MSP v1 IDs
- byte order: little-endian
- image origin: top-left
- lock coordinates are always seeker-image pixel coordinates
- the FC-side source of truth is:
  - [camera_lock.h](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.h)
  - [camera_lock.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.c)

There are four relevant state layers:

1. seeker config
2. optional FPV config
3. raw lock state
4. FC-derived lock state

## Current Flow

### 1. Seeker boot and registration

When the seeker is ready, it sends:

- `MSP_SET_SEEKER_CAM_INFO`

The FC:

- parses the seeker payload
- stores the seeker config
- resolves the MSP source/port
- binds the seeker to that source

Then:

- if the seeker does **not** require FPV projection, the FC starts polling immediately
- if the seeker **does** require FPV projection, the FC waits for `MSP_SET_FPV_CAM_INFO` before polling

Normal MSP success handling is the acknowledgment. There is no custom ACK command.

### 2. Optional FPV config

If seeker flags include `SEEKER_CAM_INFO_FLAG_FPV_PROJECTION_REQUIRED`, the FC waits for:

- `MSP_SET_FPV_CAM_INFO`

Behavior:

- FC does not poll `MSP_CAMERA_GET_LOCK` yet
- FC waits up to `500 ms`
- if FPV config does not arrive in time, the FC raises the OSD warning:
  - `NO FPV CFG`
- polling remains disabled until valid FPV config arrives

As soon as valid FPV config arrives:

- the warning clears
- polling starts

### 3. FC polling

After startup requirements are satisfied, the FC polls only the bound seeker port with:

- `MSP_CAMERA_GET_LOCK`

Request payload:

```c
// empty
```

Poll period:

- `poll_period_us = 1000000 / lock_rate_hz`

The FC sends a new poll only when:

- seeker config is valid
- the seeker source is bound
- `lock_rate_hz > 0`
- one full poll period elapsed
- no previous poll is still outstanding
- if FPV projection is required, FPV config is valid

### 4. Seeker reply

The seeker replies to `MSP_CAMERA_GET_LOCK` with:

- `flags`
- `x_px`
- `y_px`

The FC accepts replies only from the currently bound seeker source.

## Timeout Policy

The polling timeout behavior is still two-stage:

- short timeout = `2 x poll period`
- long timeout = `10 x poll period`

### Short timeout

If the FC does not receive a valid lock reply in `2 x poll period`:

- cached lock loses `HEALTHY`
- `FRESH` becomes false
- the overlay disappears automatically
- binding is kept
- polling continues

### Long timeout

If the FC does not receive a valid lock reply in `10 x poll period`:

- raw lock is cleared
- config and binding state are reset
- polling stops
- the FC waits for a new `MSP_SET_SEEKER_CAM_INFO`

## Data Structures

### Common intrinsics

```c
typedef struct cameraIntrinsics_s {
    uint32_t fx_px_x1000;
    uint32_t fy_px_x1000;
    uint32_t cx_px_x1000;
    uint32_t cy_px_x1000;
} cameraIntrinsics_t;
```

### Seeker config

```c
typedef struct seekerCamInfo_s {
    uint16_t width_px;
    uint16_t height_px;
    cameraIntrinsics_t intrinsics;
    uint8_t hfov_deg;
    uint8_t vfov_deg;
    int8_t tilt_angle_deg;
    uint8_t orientation;
    uint16_t lock_rate_hz;
    uint8_t flags;
} seekerCamInfo_t;
```

### FPV config

```c
typedef struct fpvCamInfo_s {
    cameraIntrinsics_t intrinsics;
    int8_t tilt_angle_deg;
    uint8_t flags;
} fpvCamInfo_t;
```

## Flags

### Seeker flags

Defined in [camera_lock.h](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.h):

- bit0 = `SEEKER_CAM_INFO_FLAG_VALID`
- bit1 = `SEEKER_CAM_INFO_FLAG_CROPPED`
- bit2 = `SEEKER_CAM_INFO_FLAG_INTRINSICS_VALID`
- bit3 = `SEEKER_CAM_INFO_FLAG_FOV_VALID`
- bit4 = `SEEKER_CAM_INFO_FLAG_HEALTHY`
- bit5 = `SEEKER_CAM_INFO_FLAG_FPV_PROJECTION_REQUIRED`

### FPV flags

- bit0 = `FPV_CAM_INFO_FLAG_VALID`
- bit1 = `FPV_CAM_INFO_FLAG_INTRINSICS_VALID`

### Raw lock flags

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`

### FC-derived lock flags

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`
- bit2 = `CAMERA_LOCK_FLAG_FRESH`

## Message 193: `MSP_SET_SEEKER_CAM_INFO`

### Meaning

Seeker-to-FC startup registration/configuration message.

### Direction

- sender: seeker
- receiver: FC

### Payload

```c
uint16_t width_px;
uint16_t height_px;
uint32_t fx_px_x1000;
uint32_t fy_px_x1000;
uint32_t cx_px_x1000;
uint32_t cy_px_x1000;
uint8_t hfov_deg;
uint8_t vfov_deg;
int8_t tilt_angle_deg;
uint8_t orientation;
uint16_t lock_rate_hz;
uint8_t flags;
```

Python `struct.pack` format:

```python
'<HHIIIIBBbHB'
```

### Payload size

- `27 bytes`

### Result

On success, the FC:

- stores seeker config
- binds the seeker port
- either starts polling immediately
- or waits for FPV config if `SEEKER_CAM_INFO_FLAG_FPV_PROJECTION_REQUIRED` is set

## Message 190: `MSP_SEEKER_CAM_INFO`

### Meaning

Reads back the seeker config currently cached in the FC.

### Direction

- sender: FC
- receiver: external client/tool

### Payload

Same as `MSP_SET_SEEKER_CAM_INFO`.

### Payload size

- `27 bytes`

## Message 195: `MSP_SET_FPV_CAM_INFO`

### Meaning

Writes FPV projection config into the FC.

### Direction

- sender: external config source / bridge
- receiver: FC

### Payload

```c
uint32_t fx_px_x1000;
uint32_t fy_px_x1000;
uint32_t cx_px_x1000;
uint32_t cy_px_x1000;
int8_t tilt_angle_deg;
uint8_t flags;
```

Python `struct.pack` format:

```python
'<IIIIbB'
```

### Payload size

- `18 bytes`

### Result

If the seeker required FPV projection and was waiting for this config:

- the FC clears `NO FPV CFG`
- polling starts immediately

## Message 194: `MSP_FPV_CAM_INFO`

### Meaning

Reads back the FPV projection config cached in the FC.

### Payload

Same as `MSP_SET_FPV_CAM_INFO`.

### Payload size

- `18 bytes`

## Message 191: `MSP_CAMERA_GET_LOCK`

### Meaning

This command still has two roles:

1. FC -> seeker poll request
2. FC -> external client raw-lock readback

### Poll request payload

```c
// empty
```

### Seeker reply payload

```c
uint8_t flags;
uint16_t x_px;
uint16_t y_px;
```

Python `struct.pack` format:

```python
'<BHH'
```

### Payload size

- `5 bytes`

## Message 192: `MSP_CAMERA_LOCK`

### Meaning

Reads FC-derived lock state.

### Payload

```c
uint8_t flags;
uint16_t x_px;
uint16_t y_px;
uint16_t age_ms;
```

### Payload size

- `7 bytes`

## OSD Behavior

The private overlay in:

- [osd_elements.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/osd/osd_elements.c)

uses two display modes:

1. read FC-derived lock state
2. require `detected && healthy && fresh`
3. if valid FPV projection config exists:
   - project seeker `x_px`, `y_px` into FPV image coordinates using seeker/FPV intrinsics and mount rotation
   - clamp the projected FPV pixel to the visible FPV image bounds derived from `cx`, `cy`
   - map that clamped FPV pixel onto the phased OSD grid
   - draw the live center lock sprite from that projected point
   - draw four additional cached corner sprites from the projected seeker image corners
4. otherwise, for seeker-only systems with no FPV projection path:
   - use seeker `width_px`, `height_px`, `x_px`, `y_px`
   - map directly onto the phased OSD grid
   - draw the live center lock sprite
   - draw four additional cached corner sprites from the seeker image corners
5. render corner sprites first, then the center sprite last

Current milestone behavior:

- seeker-only systems keep the current direct seeker mapping
- if valid FPV projection config exists, the overlay uses projected FPV placement
- projection-required systems suppress polling and the overlay until FPV config exists
- if projection math fails for a frame, the overlay is suppressed for that frame
- seeker-corner projections are cached and only rebuilt when seeker/FPV config changes

## PC Bridge Text Protocol

The current PC->Teensy bridge mode uses newline-terminated ASCII:

### Seeker config

```text
SEEKER width height fx fy cx cy hfov vfov tilt orientation rate flags
```

Legacy alias currently accepted during migration:

```text
INFO width height fx fy cx cy hfov vfov tilt orientation rate flags
```

### FPV config

```text
FPV fx fy cx cy tilt flags
```

### Lock updates

```text
LOCK flags x y
```

### Teensy status lines

The Teensy prints back status lines such as:

- `STATUS SEEKER_OK`
- `STATUS SEEKER_ERR <reason>`
- `STATUS FPV_OK`
- `STATUS FPV_ERR <reason>`
- `STATUS FC_SET_SEEKER_SENT`
- `STATUS FC_SET_SEEKER_ACK`
- `STATUS FC_SET_SEEKER_TIMEOUT`
- `STATUS FC_SET_SEEKER_REJECTED`
- `STATUS FC_SET_FPV_SENT`
- `STATUS FC_SET_FPV_ACK`
- `STATUS FC_SET_FPV_TIMEOUT`
- `STATUS FC_SET_FPV_REJECTED`
- `STATUS WAIT_FPV`
- `STATUS LOCK_OK flags=<n> x=<n> y=<n>`
- `STATUS FC_POLL`
- `STATUS FC_REPLY_SENT flags=<n> x=<n> y=<n>`

## Recommended Usage For Another Agent

### Implementing the seeker/peripheral side

1. boot the seeker
2. send `MSP_SET_SEEKER_CAM_INFO`
3. if projection is required, send `MSP_SET_FPV_CAM_INFO`
4. wait for normal MSP ACK behavior
5. then answer empty `MSP_CAMERA_GET_LOCK` requests with:
   - `flags`
   - `x_px`
   - `y_px`

### Implementing a debug client against the FC

1. request `MSP_SEEKER_CAM_INFO`
2. request `MSP_FPV_CAM_INFO`
3. request `MSP_CAMERA_GET_LOCK` for raw cached lock
4. request `MSP_CAMERA_LOCK` for FC-derived state

### Implementing an FC consumer

- trust `MSP_CAMERA_LOCK`, not the raw lock payload

## Important Notes

- seeker `width_px` / `height_px` are mandatory
- FPV projection currently uses:
  - seeker intrinsics + size
  - seeker tilt + orientation
  - FPV intrinsics
  - FPV tilt
- FPV `width_px` / `height_px` are intentionally omitted in this design
- when FPV projection is active, the FC derives a visible FPV image rectangle from the principal point:
  - `width ~= 2 * cx + 1`
  - `height ~= 2 * cy + 1`
- FPV orientation is assumed fixed landscape and omitted
- `lock_rate_hz == 0` means polling does not start
- `tilt_angle_deg` fields are signed
- seeker `orientation` uses:
  - `0` = `0 deg`
  - `1` = `+90 deg`
  - `2` = `180 deg`
  - `3` = `-90 deg`
- intrinsics are fixed-point `x1000`
- one seeker source is supported at a time

## Code Ownership

- camera state, binding, startup gating, polling, timeouts:
  - [camera_lock.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.c)
- MSP command parsing and serialization:
  - [msp.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/msp/msp.c)
- MSP serial source/port plumbing:
  - [msp_serial.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/msp/msp_serial.c)
- private lock overlay rendering:
  - [osd_elements.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/osd/osd_elements.c)
- `NO FPV CFG` warning:
  - [osd_warnings.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/osd/osd_warnings.c)
