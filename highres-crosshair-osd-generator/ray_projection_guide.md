# Pixel -> Ray -> Projection & Angles (STM32-Oriented Guide)

## Purpose

This guide describes the math we need for the `FINAL` camera-lock feature in this repo.

It covers:

1. seeker pixel -> FPV projection
2. seeker pixel -> body-frame direction
3. seeker pixel -> Betaflight earth-frame heading and elevation

It is written for the simplified system assumptions we currently use:

- negligible translation between cameras
- rotation-only relationship between seeker and FPV
- seeker camera and FPV camera each have intrinsics
- camera mounting difference is currently modeled from tilt and orientation

Implementation preference:

- use Betaflight's native body and earth conventions as the primary working coordinates
- this keeps the projection path aligned with Betaflight's own `rMat` and minimizes extra sign flips

## What This Guide Assumes

### Camera frame axes

For the camera frame:

- `x_cam` = right
- `y_cam` = down
- `z_cam` = forward / depth

So a pixel back-projects to a camera-frame ray of the form:

- `[x_cam, y_cam, z_cam] = [x, y, 1]`

after intrinsics normalization.

### Betaflight body / earth reference axes

For implementation in this project, use Betaflight's native reference frames:

#### Body frame

- `X_body` = forward
- `Y_body` = left
- `Z_body` = up

#### Earth frame

- `X_earth` = north
- `Y_earth` = west
- `Z_earth` = up

So the natural Betaflight earth frame is:

- `NWU`

This is **not** the same axis layout as the camera frame, so an explicit camera -> body axis remap is required.

### Camera mounting interpretation

- camera tilt angle is a rotation about the **body pitch axis**
- seeker camera orientation (`0`, `90`, `-90`, `180`) is an image rotation that corresponds to:
  - rotation around the camera optical axis in camera coordinates
  - equivalently, a roll-style orientation change of the mounted camera in the body reference interpretation

### Earth-frame angles

If we want true world-referenced heading/elevation:

- body-frame direction alone is **not enough**
- we must additionally rotate the body-frame ray using the aircraft attitude rotation matrix into Betaflight earth frame

So:

- seeker -> body-frame angles uses only seeker geometry/mounting
- seeker -> Betaflight earth-frame angles uses seeker geometry/mounting **plus FC attitude**

## Betaflight Attitude Matrix Convention

Betaflight already maintains the attitude rotation matrix:

- `rMat`

in [imu.c](/c:/Users/tamipinhasi/Documents/repos/betaflight/src/main/flight/imu.c).

From the code and comments:

- `rMat` maps **body frame -> Betaflight earth frame**
- it is used as `BF -> EF`

### Betaflight earth frame

Betaflight EF uses:

- `X_ef` = north
- `Y_ef` = west
- `Z_ef` = up

So Betaflight earth frame is effectively:

- `NWU`

### Betaflight body frame implied by `rMat`

At zero attitude, body frame aligns with Betaflight earth frame, so the body convention used by `rMat` is:

- `X_body_bf` = forward
- `Y_body_bf` = left
- `Z_body_bf` = up

That is the body convention we should use when building seeker and FPV mount rotations.

## Core Pipeline

### A. Seeker pixel -> seeker camera ray

Given seeker intrinsics:

- `fx`
- `fy`
- `cx`
- `cy`

Precompute:

- `inv_fx = 1 / fx`
- `inv_fy = 1 / fy`

Per pixel `(u, v)`:

- `x = (u - cx) * inv_fx`
- `y = (v - cy) * inv_fy`

Seeker camera ray:

- `r_seek_cam = [x, y, 1]`

This is the ray in the **camera frame**:

- right
- down
- forward

No normalization is required for projection.

## Step 1: Camera Axes -> Betaflight Body Axes

Because the camera and body axes differ, we first map the raw camera ray into a body-aligned camera-mount frame.

Using the conventions above:

- camera right -> negative Betaflight body Y
- camera down -> negative body up
- camera forward -> body forward

So the base axis remap is:

- `x_body_like =  z_cam`
- `y_body_like = -x_cam`
- `z_body_like = -y_cam`

So from:

- `r_seek_cam = [x, y, 1]`

we get:

- `r_seek_mount0 = [1, -x, -y]`

This is the seeker ray before applying the seeker mounting rotations, already expressed in Betaflight body-axis convention.

## Step 2: Apply Seeker Mount Rotation

We then apply the seeker mounting rotation to move from the body-aligned mount frame into the actual body frame.

This rotation is built from:

- seeker tilt angle
- seeker orientation

### Seeker tilt

The seeker tilt angle is a rotation around the **body pitch axis**.

In the current body-frame convention, that is the sideways axis:

- rotation about `y_body`

Call this matrix:

- `R_seek_tilt`

### Seeker orientation

The seeker orientation enum represents discrete camera rotation:

- `0`
- `+90`
- `-90`
- `180`

Conceptually this is:

- rotation about the camera optical axis in camera coordinates

After the axis remap, we treat it as the corresponding discrete rotation that reorients the image before the ray is expressed in the body frame.

Call this matrix:

- `R_seek_orient`

### Combined seeker-to-body rotation

So the seeker body-frame ray is:

- `r_body = R_seek_mount * r_seek_cam`

where:

- `R_seek_mount = R_seek_tilt * R_seek_orient * R_axis_map`

and:

- `R_axis_map` performs the camera-frame -> body-frame axis conversion

The exact multiplication order should follow the implementation convention, but the important point is:

1. convert axes
2. apply orientation
3. apply tilt

## Step 2A: Build The Static Mount Rotation Matrices From Info Data

This is the part that should be precomputed from the MSP config messages.

### Base camera-to-body alignment

First define the fixed camera-axis -> Betaflight-body-axis alignment:

```text
R_axis_map =
[
  [ 0,  0,  1 ],
  [ -1, 0,  0 ],
  [ 0, -1,  0 ]
]
```

This matrix maps:

- camera forward -> body forward
- camera right   -> negative body Y
- camera down    -> negative body Z

So:

```text
r_body_like = R_axis_map * r_cam
```

### Seeker orientation matrix

The seeker `orientation` field is a static rotation around the optical axis.

After `R_axis_map`, the optical axis is aligned with Betaflight body `X`, so we can represent seeker orientation as a rotation about `X_body`.

Use these discrete matrices:

#### `orientation = 0`

```text
R_orient(0) = I
```

#### `orientation = 90`

```text
R_orient(+90) =
[
  [ 1, 0,  0 ],
  [ 0, 0, -1 ],
  [ 0, 1,  0 ]
]
```

#### `orientation = 180`

```text
R_orient(180) =
[
  [ 1,  0,  0 ],
  [ 0, -1,  0 ],
  [ 0,  0, -1 ]
]
```

#### `orientation = -90`

```text
R_orient(-90) =
[
  [ 1,  0, 0 ],
  [ 0,  0, 1 ],
  [ 0, -1, 0 ]
]
```

### Tilt matrix

Both seeker and FPV tilt are rotations around the Betaflight body pitch axis:

- `Y_body`

Use a standard active rotation about `Y_body`:

```text
R_tilt(theta) =
[
  [ cos(theta), 0, sin(theta) ],
  [ 0,          1, 0          ],
  [ -sin(theta),0, cos(theta) ]
]
```

where:

- `theta = degreesToRadians(tilt_angle_deg)`

### Final seeker mount matrix

From seeker info:

- `tilt_angle_deg`
- `orientation`

build:

```text
R_body_from_seeker =
    R_tilt(seeker_tilt_angle_deg)
  * R_orient(seeker_orientation)
  * R_axis_map
```

This matrix converts a ray from seeker camera frame directly into Betaflight body frame:

```text
r_body = R_body_from_seeker * r_seek_cam
```

### Final FPV mount matrix

From FPV info:

- `tilt_angle_deg`

build:

```text
R_body_from_fpv =
    R_tilt(fpv_tilt_angle_deg)
  * R_axis_map
```

Because FPV orientation is assumed fixed landscape in this design, there is no FPV orientation matrix.

This matrix converts a ray from FPV camera frame into Betaflight body frame:

```text
r_body = R_body_from_fpv * r_fpv_cam
```

## Step 2B: Build The Static Relative Rotation Between Seeker And FPV

Under the current assumption:

- relative translation is neglected
- only relative rotation matters

Once both camera mount matrices are known, the static relative rotation from seeker camera frame to FPV camera frame is:

```text
R_fpv_from_seeker =
    (R_body_from_fpv)^T
  * R_body_from_seeker
```

because:

- `R_body_from_seeker` maps seeker camera -> body
- `R_body_from_fpv` maps FPV camera -> body
- for pure rotation, inverse = transpose

So:

```text
r_fpv_cam = R_fpv_from_seeker * r_seek_cam
```

This is the key static matrix for seeker-to-FPV projection.

It should be precomputed once whenever:

- seeker config changes
- FPV config changes

## Step 3A: Seeker -> FPV Projection

To project onto the FPV camera, we need the relative rotation between seeker and FPV.

Because translation is neglected, this is rotation-only.

We compute:

- seeker ray in seeker camera frame: `r_seek_cam`
- rotate directly into FPV camera frame with the precomputed static relative rotation:

```text
r_fpv_cam = R_fpv_from_seeker * r_seek_cam
```

This gives:

- `r_fpv_cam = [X2, Y2, Z2]`

in FPV camera coordinates.

Then project using FPV intrinsics:

- `u2 = fx2 * (X2 / Z2) + cx2`
- `v2 = fy2 * (Y2 / Z2) + cy2`

Important:

- this gives an FPV-image pixel coordinate
- the final OSD quantization step happens after this projection

### Projection validity and clamp policy

For the current Betaflight implementation:

- if `Z2 <= epsilon`, treat the projection as invalid for that frame
  - this means the projected ray is behind the FPV camera or numerically unstable
  - do **not** draw the lock for that frame

If the projected point is in front of the FPV camera, we still clamp it to a visible image rectangle before converting to OSD coordinates.

Because the current FPV config intentionally does **not** include `width_px` / `height_px`, the implementation derives the visible FPV rectangle from the principal point under the centered-principal-point assumption:

- `fpv_width_px  = round(2 * cx2 + 1)`
- `fpv_height_px = round(2 * cy2 + 1)`

Then clamp:

- `u2_clamped = clamp(u2, 0, fpv_width_px - 1)`
- `v2_clamped = clamp(v2, 0, fpv_height_px - 1)`

This is the projected pixel that feeds the OSD lock overlay.

### OSD consumer path

The projected FPV pixel is the direct input to the existing phased lock overlay:

1. seeker raw lock pixel -> projected FPV pixel
2. projected FPV pixel -> clamped FPV pixel
3. clamped FPV pixel -> virtual OSD sub-glyph coordinate
4. virtual OSD coordinate -> phased `3x2` lock glyph placement

So the OSD element no longer uses the seeker image dimensions whenever a valid FPV projection path is active.

## Step 3B: Seeker -> Betaflight Body-Frame Angles

If we only want body-frame line-of-sight angles, use `r_body = [Xb, Yb, Zb]`.

With Betaflight body convention:

- `x_body` = forward
- `y_body` = left
- `z_body` = up

Body-frame heading/elevation are then:

- `heading_body   = atan2(Yb, Xb)`
- `elevation_body = atan2(Zb, sqrt(Xb^2 + Yb^2))`

These are **Betaflight body-referenced** angles, not earth-frame angles.

## Step 3C: Seeker -> Betaflight Earth Frame Angles

For world-referenced angles using Betaflight attitude, we must rotate the body-frame ray by the FC attitude rotation matrix.

If:

- `rMat`

is the Betaflight attitude rotation matrix, then:

- `r_ef = rMat * r_body`

Let:

- `r_ef = [Xef, Yef, Zef]`

This gives the ray in Betaflight earth frame:

- north / west / up

This should be the default implementation target, because it matches the FC directly and avoids extra conversions.

If Betaflight-EF angles are sufficient, use:

- `heading_ef   = atan2(Yef, Xef)`
- `elevation_ef = atan2(Zef, sqrt(Xef^2 + Yef^2))`

This is the correct place to use the FC attitude.

So the important correction is:

- seeker pixel -> Betaflight earth angles is **not** just camera intrinsics + camera mount
- it is:
  - seeker intrinsics
  - seeker mount rotation
  - aircraft attitude rotation into Betaflight EF

## Performance Strategy

The STM32-oriented performance idea is still the same:

- do not normalize the ray unless needed
- precompute inverse focal lengths
- precompute rotation matrices when config changes
- avoid `atan2f` unless the output really must be an angle

For projection:

- normalization is not needed
- only division by `Z` is needed for pinhole projection

For control loops:

- small-angle proxies may be enough in many cases

## Fast Angle Strategy

`atan2f` is expensive.

If exact angles are required:

- use a fast approximation or hybrid scheme

If only a control proxy is required:

- use ray components directly
- for small angles:
  - heading proxy ~= `Y / X`
  - elevation proxy ~= `Z / X`
  - or use the normalized image-space `x`, `y` directly when acceptable

## What Must Be Precomputed

When config arrives, precompute once:

- `inv_fx_seek`
- `inv_fy_seek`
- `inv_fx_fpv`
- `inv_fy_fpv`
- seeker mount rotation matrix
- FPV mount rotation matrix
- optionally relative seeker-to-FPV rotation
- optionally combined:
  - `R_fpv_from_seek = R_fpv_from_body * R_seek_from_cam`

Then per lock sample, runtime work is:

1. pixel -> seeker ray
2. seeker ray -> Betaflight body ray
3. body ray -> FPV ray or Betaflight earth ray
4. projection or angle extraction

## Final Takeaway

The corrected system-specific pipeline is:

1. seeker pixel -> normalized seeker camera ray
2. camera-frame axes -> Betaflight body-frame axes
3. apply seeker orientation + tilt
4. get Betaflight body-frame ray
5. optionally apply FPV inverse mount rotation and project to FPV image
6. optionally apply FC attitude rotation using Betaflight `rMat` to get EF ray
7. compute final Betaflight earth-frame heading + elevation

This is the right guide for our case because it now explicitly includes:

- different camera/body axes
- Betaflight-native body/earth conventions
- seeker mounting interpretation
- FPV projection path
- Betaflight `rMat` convention
- FC attitude requirement for true earth-frame angles
