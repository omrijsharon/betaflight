# Sub-Glyph Resolution Painter Guide

## Purpose

This guide explains to another coding agent how to write a Python script that generates a `288 x 72` BMP for the analog OSD font sheet.

The BMP is a `24 x 4` glyph sheet:

- glyph width: `12 px`
- glyph height: `18 px`
- total bitmap size: `24 * 12 = 288 px` by `4 * 18 = 72 px`

Only the **left 9 glyph columns** are used for the custom crosshair glyph bank.

- left custom area: `9 x 4 glyphs = 36 glyphs`
- right logo area: `15 x 4 glyphs = 60 glyphs`

The right side must be preserved for the Betaflight logo. The Python script should either:

1. load an existing `288 x 72` BMP and overwrite only glyph columns `0..8`, or
2. create a blank green image and leave glyph columns `9..23` untouched for a later logo pass.

## Colors

Only these 3 RGB colors are allowed:

- transparent: green = `(0, 255, 0)`
- black = `(0, 0, 0)`
- white = `(255, 255, 255)`

Every pixel not used by the crosshair must stay green.

## Definitions

### Glyph

A glyph is one OSD character tile:

- size: `12 x 18 px`

### Sprite

The crosshair is not a single glyph. It is a larger object built from multiple glyphs.

For this design, the sprite is:

- `3 glyphs` wide
- `2 glyphs` tall
- total sprite size: `36 x 36 px`

### Phase

A phase is one shifted version of the same sprite.

This design uses:

- `2` horizontal phases
- `3` vertical phases
- total phases: `2 * 3 = 6`

The "fake higher resolution" comes from switching between these pre-rendered shifted phases.

Important:

- a phase is **not** a visible section of one glyph
- a phase is one whole `36 x 36` sprite state
- each phase is sliced into `3 x 2 = 6` glyphs

So the total glyph count is:

- `6 phases * 6 tiles per phase = 36 glyphs`

## Crosshair Geometry

### Overall icon

Inside each `36 x 36` sprite, draw a `24 x 24` crosshair icon.

The icon is a plus sign with a hollow center:

- icon size: `24 x 24 px`
- center hole size: `12 x 12 px`

This means:

- horizontal arm length on each side of the hole: `6 px`
- vertical arm length on each side of the hole: `6 px`

### Line style

Each arm is made from adjacent parallel white and black lines to create a lit bevel effect with the light coming from the top-left.

Recommended interpretation:

- horizontal arm:
  - upper line = white
  - lower line = black
- vertical arm:
  - left line = white
  - right line = black

This gives the classic "top-left highlight / bottom-right shadow" look.

### Exact 24 x 24 icon drawing recommendation

Use local icon coordinates `x = 0..23`, `y = 0..23`.

Center hole:

- hole rectangle: `x = 6..17`, `y = 6..17`
- this is exactly `12 x 12`

Arms:

- top vertical arm:
  - white pixels at `x = 11`, `y = 0..5`
  - black pixels at `x = 12`, `y = 0..5`
- bottom vertical arm:
  - white pixels at `x = 11`, `y = 18..23`
  - black pixels at `x = 12`, `y = 18..23`
- left horizontal arm:
  - white pixels at `y = 11`, `x = 0..5`
  - black pixels at `y = 12`, `x = 0..5`
- right horizontal arm:
  - white pixels at `y = 11`, `x = 18..23`
  - black pixels at `y = 12`, `x = 18..23`

All other icon pixels remain green.

This produces a symmetric hollow-center plus sign.

## Why the icon is 24 x 24 inside a 36 x 36 sprite

The extra space is the motion margin.

The sprite is larger than the icon so the icon can shift between phases without clipping.

For this design:

- sprite size: `36 x 36`
- icon size: `24 x 24`
- available spare space: `12 px` horizontally and `12 px` vertically

That margin is what makes the sub-glyph trick possible.

## Phase Offsets

This design targets about `6 px` fake movement resolution.

Recommended phase offsets:

- horizontal offsets: `x = [3, 9]`
- vertical offsets: `y = [0, 6, 12]`

Why this is a good choice:

- horizontal phase step = `6 px`
- vertical phase step = `6 px`
- the full set of phase centers stays visually centered inside the `36 x 36` sprite

Using these offsets, place the `24 x 24` icon inside the `36 x 36` sprite canvas at:

- phase `(0, 0)` -> `(3, 0)`
- phase `(1, 0)` -> `(9, 0)`
- phase `(0, 1)` -> `(3, 6)`
- phase `(1, 1)` -> `(9, 6)`
- phase `(0, 2)` -> `(3, 12)`
- phase `(1, 2)` -> `(9, 12)`

The script should render the same icon 6 times, once at each phase offset.

## How to slice each phase into glyphs

Each phase sprite is `36 x 36`.

Slice it into `3 x 2` glyph tiles:

- tile columns:
  - `0`: `x = 0..11`
  - `1`: `x = 12..23`
  - `2`: `x = 24..35`
- tile rows:
  - `0`: `y = 0..17`
  - `1`: `y = 18..35`

That gives 6 glyphs per phase:

- top-left
- top-center
- top-right
- bottom-left
- bottom-center
- bottom-right

## Packing the 36 glyphs into the BMP

The left custom area is exactly `9 glyph columns x 4 glyph rows`, which is `108 x 72 px`.

Pack the 6 phases into this left area as six `3 x 2` phase blocks.

Recommended packing:

- phase index = `phase_y * 2 + phase_x`
- phase block column = `phase_index % 3`
- phase block row = `phase_index // 3`

This uses a `3 x 2` layout of phase blocks, which fits exactly:

- block width = `3 glyphs`
- block height = `2 glyphs`
- total grid = `3 blocks across x 2 blocks down`

So the left area is filled exactly:

- width: `3 blocks * 3 glyphs = 9 glyph columns`
- height: `2 blocks * 2 glyphs = 4 glyph rows`

### Concrete phase block layout

Use this mapping:

- phase `(0, 0)` -> block `(0, 0)`
- phase `(1, 0)` -> block `(1, 0)`
- phase `(0, 1)` -> block `(2, 0)`
- phase `(1, 1)` -> block `(0, 1)`
- phase `(0, 2)` -> block `(1, 1)`
- phase `(1, 2)` -> block `(2, 1)`

Within each block, tiles are placed row-major:

- top row: tile columns `0, 1, 2`
- bottom row: tile columns `0, 1, 2`

## Pixel placement in the final BMP

For a glyph at global glyph coordinates `(glyph_col, glyph_row)`:

- destination pixel x = `glyph_col * 12`
- destination pixel y = `glyph_row * 18`

Only write into glyph columns `0..8`.

Do not modify glyph columns `9..23`.

## Recommended script structure

The Python script should roughly do this:

1. Open an existing `288 x 72` BMP if preserving a logo, otherwise create a new RGB image filled with green.
2. Define constants:
   - glyph size `12 x 18`
   - sheet size `24 x 4 glyphs`
   - sprite size `36 x 36`
   - icon size `24 x 24`
   - phase offsets as listed above
3. For each of the 6 phases:
   - create a temporary `36 x 36` green sprite canvas
   - draw the `24 x 24` crosshair icon at the phase offset
   - slice the sprite into `3 x 2` glyph tiles
   - copy each tile into the proper place in the left `9 x 4` glyph region
4. Save as BMP.

## Important implementation details

- Keep the image in plain RGB, not RGBA.
- Do not anti-alias.
- Do not use any colors except the exact three allowed colors.
- Keep all unused pixels green.
- The center hole must remain fully green.
- The right 15 glyph columns are reserved for the logo and should be preserved.

## What this gives at runtime

At runtime, Betaflight does not redraw pixels.

Instead, it chooses one of 6 pre-rendered phases and draws the corresponding `3 x 2` glyph block.

That is how the crosshair appears to move with about `6 px` fake sub-glyph resolution even though the analog OSD is still character-cell based.

## What another agent must not misunderstand

- The sprite is `36 x 36`, not `24 x 24`.
- The icon is `24 x 24` and lives inside the sprite.
- The empty center is `12 x 12`.
- There are `6` total phases, not `6` phases per axis.
- There are `36` total glyphs because each phase uses `6` glyphs.
- The higher resolution comes from phase switching, not from changing glyph pixels at runtime.

## Betaflight Glyph ID Mapping

The BMP cells used for the custom crosshair bank are in the left 9 columns of the old logo area.

Because the logo area is still stored as a `4 x 24` row-major tile block starting at glyph `160`, the 36 custom glyphs are **not** one contiguous glyph-ID range.

The exact mapping is:

- BMP row `0`, cols `0..8` -> glyph IDs `160..168` -> `0xA0..0xA8`
- BMP row `1`, cols `0..8` -> glyph IDs `184..192` -> `0xB8..0xC0`
- BMP row `2`, cols `0..8` -> glyph IDs `208..216` -> `0xD0..0xD8`
- BMP row `3`, cols `0..8` -> glyph IDs `232..240` -> `0xE8..0xF0`

Another agent implementing the OSD runtime side must use this exact mapping when choosing glyphs.

### Recommended phase-to-glyph mapping

The guide above packs the 6 phases into six `3 x 2` blocks in the left `9 x 4` region.

That means each phase owns 6 glyph IDs.

Using the recommended phase-block layout:

- phase `(0, 0)` -> glyphs:
  - top row: `160, 161, 162`
  - bottom row: `184, 185, 186`
- phase `(1, 0)` -> glyphs:
  - top row: `163, 164, 165`
  - bottom row: `187, 188, 189`
- phase `(0, 1)` -> glyphs:
  - top row: `166, 167, 168`
  - bottom row: `190, 191, 192`
- phase `(1, 1)` -> glyphs:
  - top row: `208, 209, 210`
  - bottom row: `232, 233, 234`
- phase `(0, 2)` -> glyphs:
  - top row: `211, 212, 213`
  - bottom row: `235, 236, 237`
- phase `(1, 2)` -> glyphs:
  - top row: `214, 215, 216`
  - bottom row: `238, 239, 240`

These are the 6 glyph triplets the OSD code will need to draw for each phase.

## Runtime Mapping Notes

This section is not needed to paint the BMP, but it is needed for the Betaflight OSD code that will later use these glyphs.

### Coarse position vs phase

At runtime, the crosshair position should be represented in a finer virtual coordinate system.

Recommended interpretation:

- one horizontal phase step = `6 px`
- one vertical phase step = `6 px`

So the runtime code can represent crosshair position in virtual units where:

- `phaseX = virtual_x % 2`
- `phaseY = virtual_y % 3`
- coarse cell X advances every 2 horizontal virtual steps
- coarse cell Y advances every 3 vertical virtual steps

Equivalent view in pixels:

- coarse cell width = `12 px`
- coarse cell height = `18 px`
- horizontal phase step = `6 px`
- vertical phase step = `6 px`

### Recommended runtime selection formula

For a desired virtual position:

- `phaseX = vx % 2`
- `phaseY = vy % 3`
- `baseCellX = vx / 2`
- `baseCellY = vy / 3`

Then:

- choose the phase block from `(phaseX, phaseY)`
- draw the `3 x 2` glyph block starting at `(baseCellX, baseCellY)`

That means the OSD renderer writes 6 glyphs:

- `(baseCellX + 0, baseCellY + 0)`
- `(baseCellX + 1, baseCellY + 0)`
- `(baseCellX + 2, baseCellY + 0)`
- `(baseCellX + 0, baseCellY + 1)`
- `(baseCellX + 1, baseCellY + 1)`
- `(baseCellX + 2, baseCellY + 1)`

using the glyph IDs listed in the phase table above.

### Why this produces fake sub-glyph movement

When the desired crosshair moves by one virtual step:

- sometimes only `phaseX` or `phaseY` changes
- the coarse OSD cell position does not change yet
- the displayed glyph block changes to a shifted pre-rendered phase

After enough virtual steps:

- the coarse cell position increments
- the phase wraps around

That is what produces smoother apparent movement on a grid-based analog OSD.
