# Camera Lock MSP Guide

## Purpose

This file explains the implemented camera-lock MSP flow in this Betaflight fork.

The camera is treated as a private MSP peripheral:

- it boots on its own
- when ready, it sends camera info to the FC
- the FC binds the camera to that MSP source/port
- the FC then polls that same port for lock data at the configured rate
- the FC caches the result and uses it for OSD lock rendering

These commands are private fork extensions and are intended to be compiled under `USE_FINAL`.

## Command List

- `MSP_CAMERA_INFO` = `190`
- `MSP_CAMERA_GET_LOCK` = `191`
- `MSP_CAMERA_LOCK` = `192`
- `MSP_SET_CAMERA_INFO` = `193`

## General Notes

- MSP version: custom MSP v1 IDs
- payload byte order: little-endian
- image origin: top-left
- lock coordinates are image pixel coordinates in the active camera image
- the FC-side source of truth is:
  - [camera_lock.h](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.h)
  - [camera_lock.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.c)

There are three relevant layers of state:

1. camera info
   - static or slow-changing metadata
2. raw lock state
   - latest lock sample received from the camera
3. FC-derived lock state
   - raw lock plus FC-computed `age_ms` and `fresh`

## Implemented Flow

### 1. Camera boot and registration

When the camera is up and ready, it sends:

- `MSP_SET_CAMERA_INFO`

The FC:

- parses the payload
- stores the camera info
- identifies which MSP source/port sent it
- binds the camera to that source
- computes the polling period from `lock_rate_hz`
- starts polling that same port immediately

Normal MSP success handling acts as the acknowledgment.
There is no separate custom ack message.

### 2. FC polling

After a successful bind, the FC sends:

- `MSP_CAMERA_GET_LOCK`

with an empty payload to the bound camera port only.

The poll period is:

- `poll_period_us = 1000000 / lock_rate_hz`

The FC sends a new poll only when:

- camera info is valid
- a camera source is bound
- `lock_rate_hz > 0`
- one full poll period elapsed since the last sent poll
- there is no outstanding unanswered poll

### 3. Camera reply

The camera replies to `MSP_CAMERA_GET_LOCK` with:

- `flags`
- `x_px`
- `y_px`

The FC accepts the reply only if it came from the currently bound source.

The FC then:

- updates raw lock state
- updates `lastReplyReceivedUs`
- clears the outstanding-poll flag

### 4. FC-derived state and OSD

The FC computes:

- `age_ms`
- `fresh`
- `healthy`

from its own clock and timeout rules.

The private lock OSD overlay uses the FC-derived state and renders only when:

- `DETECTED`
- `HEALTHY`
- `FRESH`

are all set.

## Timeout Policy

The implemented timeout behavior is two-stage:

- short timeout = `2 x poll period`
- long timeout = `10 x poll period`

### Short timeout

If no valid reply has been received within `2 x poll period`:

- cached lock is degraded
- `HEALTHY` is cleared
- `FRESH` becomes false
- the lock overlay disappears automatically
- binding is kept
- polling continues

### Long timeout

If no valid reply has been received within `10 x poll period`:

- binding is dropped
- polling stops
- raw lock state is cleared
- the FC waits for a new `MSP_SET_CAMERA_INFO`

### Rebinding

If a new valid `MSP_SET_CAMERA_INFO` arrives from a different source:

- the new source replaces the old one immediately
- polling switches to the new source and its `lock_rate_hz`

## Message 193: `MSP_SET_CAMERA_INFO`

### Meaning

Camera-to-FC registration/configuration message.

This is the message the camera sends once it has finished booting and is ready to be polled.

### Direction

- sender: camera
- receiver: FC

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

Python packing format for this exact payload:

```python
'<HHIIIIBBbBHB'
```

Meaning:

- `H H` = `width_px`, `height_px`
- `I I I I` = `fx_px_x1000`, `fy_px_x1000`, `cx_px_x1000`, `cy_px_x1000`
- `B B` = `hfov_deg`, `vfov_deg`
- `b` = `tilt_angle_deg`
- `B` = `orientation`
- `H` = `lock_rate_hz`
- `B` = `flags`

This is the correct order currently used by the Betaflight fork and the Python fake-camera side.

### Field meanings

- `width_px`
  - active image width in pixels
- `height_px`
  - active image height in pixels
- `fx_px_x1000`
  - focal length in x, pixels times `1000`
- `fy_px_x1000`
  - focal length in y, pixels times `1000`
- `cx_px_x1000`
  - principal point x, pixels times `1000`
- `cy_px_x1000`
  - principal point y, pixels times `1000`
- `hfov_deg`
  - horizontal field of view, integer degrees
- `vfov_deg`
  - vertical field of view, integer degrees
- `tilt_angle_deg`
  - signed camera tilt angle, `-90 .. +90`
- `orientation`
  - image rotation around the front axis:
    - `0` = `0 deg`
    - `1` = `+90 deg`
    - `2` = `180 deg`
    - `3` = `-90 deg`
- `lock_rate_hz`
  - requested FC polling rate
- `flags`
  - info validity/health bits

### Result

On success, the FC:

- stores the info
- binds the camera to the source port
- starts lock polling

## Message 190: `MSP_CAMERA_INFO`

### Meaning

Reads back the currently cached camera info from the FC.

This is the FC-exported form of the info that was previously provided through `MSP_SET_CAMERA_INFO`.

### Direction

- sender: FC
- receiver: external tool/client

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

### How another agent should use it

Use this when you need to inspect the FC's current camera configuration state.

For intrinsics:

- divide `*_x1000` by `1000.0`

## Message 191: `MSP_CAMERA_GET_LOCK`

### Meaning

This command has two roles:

1. **FC -> camera**
   - empty poll request
2. **FC -> external tool**
   - read back the latest raw lock state cached in the FC

### Poll request

Request payload:

```c
// empty
```

The FC sends this request to the bound camera port only.

### Camera reply payload

```c
uint8  flags;
uint16 x_px;
uint16 y_px;
```

Python packing format:

```python
'<BHH'
```

### Field meanings

- `flags`
  - raw lock flags
- `x_px`
  - lock x coordinate in active image pixels
- `y_px`
  - lock y coordinate in active image pixels

### Raw lock flags

Defined in `camera_lock.h`:

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`

### External readback use

When an external client requests `MSP_CAMERA_GET_LOCK` from the FC, the FC returns the latest raw cached lock state.

This is useful for:

- debugging camera replies
- checking that FC caching works

It does not include FC-derived freshness or age.

## Message 192: `MSP_CAMERA_LOCK`

### Meaning

Reads back the FC-derived lock state.

This is the message other FC consumers should trust for behavior decisions because it includes FC-side aging/freshness.

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
  - cached lock x coordinate
- `y_px`
  - cached lock y coordinate
- `age_ms`
  - age of the cached sample, computed from the FC clock

### FC-derived lock flags

Defined in `camera_lock.h`:

- bit0 = `CAMERA_LOCK_FLAG_DETECTED`
- bit1 = `CAMERA_LOCK_FLAG_HEALTHY`
- bit2 = `CAMERA_LOCK_FLAG_FRESH`

### How another agent should use it

Use this for:

- lock-driven OSD behavior
- lock validity decisions
- checking whether the FC considers the lock usable

This is the preferred message when you care about:

- `detected`
- `healthy`
- `fresh`
- `age_ms`

## OSD Mapping Flow

The private camera-lock overlay in:

- [osd_elements.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/osd/osd_elements.c)

does this:

1. read FC-derived state from `camera_lock.c`
2. require `detected && healthy && fresh`
3. use:
   - `x_px`
   - `y_px`
   - `width_px`
   - `height_px`
4. map the image-space lock point into the phased OSD grid
5. quantize to the virtual OSD coordinates
6. draw the `3x2` lock sprite using phase-selected glyphs

Important:

- current OSD mapping uses `width_px` and `height_px`
- it does **not** currently use:
  - `fx/fy/cx/cy`
  - `hfov/vfov`
  - `tilt_angle_deg`
  - `orientation`

So the current implementation is:

- image-space normalization to phased OSD space

not full camera-geometry projection.

## Wire Sizes

### `MSP_SET_CAMERA_INFO`

Payload size:

- `27 bytes`

### `MSP_CAMERA_INFO`

Payload size:

- `27 bytes`

MSP v1 response frame size:

- `33 bytes`

### `MSP_CAMERA_GET_LOCK`

Payload size:

- `5 bytes`

MSP v1 response frame size:

- `11 bytes`

### `MSP_CAMERA_LOCK`

Payload size:

- `7 bytes`

MSP v1 response frame size:

- `13 bytes`

## Recommended Usage For Another Agent

If you are implementing the camera side:

1. boot the camera
2. when ready, send `MSP_SET_CAMERA_INFO`
3. wait for normal MSP success/ack behavior
4. then wait for empty `MSP_CAMERA_GET_LOCK` requests from the FC
5. reply with:
   - `flags`
   - `x_px`
   - `y_px`

If you are implementing a debug tool on the FC side:

1. request `MSP_CAMERA_INFO`
2. request `MSP_CAMERA_GET_LOCK` if you want raw cached lock
3. request `MSP_CAMERA_LOCK` if you want FC-derived validity

If you are implementing an FC consumer:

- prefer `MSP_CAMERA_LOCK` semantics, not raw lock semantics

## Important Implementation Notes

- `lock_rate_hz == 0` means polling should not start
- `width_px <= 1` or `height_px <= 1` makes OSD mapping invalid
- `tilt_angle_deg` is signed
- `orientation` and `tilt_angle_deg` are different:
  - `orientation` = image rotation
  - `tilt_angle_deg` = camera pitch/tilt
- intrinsics are fixed-point `x1000`
- one camera source is supported at a time

## Code Ownership

Current code ownership is:

- camera state, binding, polling, timeouts:
  - [camera_lock.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/sensors/camera_lock.c)
- MSP command parsing and serialization:
  - [msp.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/msp/msp.c)
- MSP serial source/port plumbing:
  - [msp_serial.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/msp/msp_serial.c)
- private lock overlay rendering:
  - [osd_elements.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/osd/osd_elements.c)
