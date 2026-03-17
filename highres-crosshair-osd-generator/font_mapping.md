# High-Res Crosshair Font Mapping

## Purpose

This file documents the 36 custom glyphs used by the high-resolution crosshair.

The glyphs occupy the **left 9 columns** of the old Betaflight logo block:

- BMP glyph columns: `0..8`
- BMP glyph rows: `0..3`

These glyphs are packed into:

- `6` phases
- each phase is one `3 x 2` glyph group

This mapping is intended for the Betaflight runtime code that will later choose:

- which phase to use
- which 6 glyphs to draw
- in what order to draw them

## Terminology

### Phase

A phase is one shifted version of the same `3 x 2` crosshair sprite.

This design uses:

- `2` horizontal phases
- `3` vertical phases

So the phase coordinates are:

- `phaseX = 0 or 1`
- `phaseY = 0, 1, or 2`

Recommended meaning:

- `phaseX = 0` -> left sub-cell position
- `phaseX = 1` -> right sub-cell position
- `phaseY = 0` -> top sub-cell position
- `phaseY = 1` -> middle sub-cell position
- `phaseY = 2` -> bottom sub-cell position

## Betaflight Glyph-ID Convention

Betaflight typically refers to glyphs as:

- decimal glyph ID
- hex glyph ID
- symbolic `SYM_...` name when one exists

For these new custom glyphs, the runtime code can use either:

- raw numeric IDs, or
- new `SYM_...` defines added to `osd_symbols.h`

This file gives:

- decimal ID
- hex ID
- logical tile name inside the phase block

## Glyph Areas Reused From the Logo

The old logo block begins at glyph `160` (`0xA0`) and is stored row-major across `24` glyph columns.

The free left-side region now corresponds to:

- row `0`, cols `0..8` -> glyphs `160..168` -> `0xA0..0xA8`
- row `1`, cols `0..8` -> glyphs `184..192` -> `0xB8..0xC0`
- row `2`, cols `0..8` -> glyphs `208..216` -> `0xD0..0xD8`
- row `3`, cols `0..8` -> glyphs `232..240` -> `0xE8..0xF0`

## Phase Packing Layout

The 6 phases are packed into six `3 x 2` blocks:

- phase `(0, 0)` -> block `(0, 0)`
- phase `(1, 0)` -> block `(1, 0)`
- phase `(0, 1)` -> block `(2, 0)`
- phase `(1, 1)` -> block `(0, 1)`
- phase `(0, 2)` -> block `(1, 1)`
- phase `(1, 2)` -> block `(2, 1)`

Within each phase block, the tile order is always:

1. top-left
2. top-center
3. top-right
4. bottom-left
5. bottom-center
6. bottom-right

## Recommended Phase Names

To keep the runtime logic readable, use these names:

- `PHASE_X0_Y0` = left-top
- `PHASE_X1_Y0` = right-top
- `PHASE_X0_Y1` = left-middle
- `PHASE_X1_Y1` = right-middle
- `PHASE_X0_Y2` = left-bottom
- `PHASE_X1_Y2` = right-bottom

If you prefer Betaflight-style symbol names, the 6 groups could be named:

- `SYM_XHAIR_PHASE_X0_Y0_*`
- `SYM_XHAIR_PHASE_X1_Y0_*`
- `SYM_XHAIR_PHASE_X0_Y1_*`
- `SYM_XHAIR_PHASE_X1_Y1_*`
- `SYM_XHAIR_PHASE_X0_Y2_*`
- `SYM_XHAIR_PHASE_X1_Y2_*`

where `*` is one of:

- `TL`
- `TC`
- `TR`
- `BL`
- `BC`
- `BR`

## Full Mapping Table

### Phase `PHASE_X0_Y0` (left-top)

Offsets:

- `phaseX = 0`
- `phaseY = 0`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 160 | `0xA0` |
| TC | 161 | `0xA1` |
| TR | 162 | `0xA2` |
| BL | 184 | `0xB8` |
| BC | 185 | `0xB9` |
| BR | 186 | `0xBA` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X0_Y0_TL = 160`
- `SYM_XHAIR_PHASE_X0_Y0_TC = 161`
- `SYM_XHAIR_PHASE_X0_Y0_TR = 162`
- `SYM_XHAIR_PHASE_X0_Y0_BL = 184`
- `SYM_XHAIR_PHASE_X0_Y0_BC = 185`
- `SYM_XHAIR_PHASE_X0_Y0_BR = 186`

### Phase `PHASE_X1_Y0` (right-top)

Offsets:

- `phaseX = 1`
- `phaseY = 0`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 163 | `0xA3` |
| TC | 164 | `0xA4` |
| TR | 165 | `0xA5` |
| BL | 187 | `0xBB` |
| BC | 188 | `0xBC` |
| BR | 189 | `0xBD` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X1_Y0_TL = 163`
- `SYM_XHAIR_PHASE_X1_Y0_TC = 164`
- `SYM_XHAIR_PHASE_X1_Y0_TR = 165`
- `SYM_XHAIR_PHASE_X1_Y0_BL = 187`
- `SYM_XHAIR_PHASE_X1_Y0_BC = 188`
- `SYM_XHAIR_PHASE_X1_Y0_BR = 189`

### Phase `PHASE_X0_Y1` (left-middle)

Offsets:

- `phaseX = 0`
- `phaseY = 1`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 166 | `0xA6` |
| TC | 167 | `0xA7` |
| TR | 168 | `0xA8` |
| BL | 190 | `0xBE` |
| BC | 191 | `0xBF` |
| BR | 192 | `0xC0` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X0_Y1_TL = 166`
- `SYM_XHAIR_PHASE_X0_Y1_TC = 167`
- `SYM_XHAIR_PHASE_X0_Y1_TR = 168`
- `SYM_XHAIR_PHASE_X0_Y1_BL = 190`
- `SYM_XHAIR_PHASE_X0_Y1_BC = 191`
- `SYM_XHAIR_PHASE_X0_Y1_BR = 192`

### Phase `PHASE_X1_Y1` (right-middle)

Offsets:

- `phaseX = 1`
- `phaseY = 1`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 208 | `0xD0` |
| TC | 209 | `0xD1` |
| TR | 210 | `0xD2` |
| BL | 232 | `0xE8` |
| BC | 233 | `0xE9` |
| BR | 234 | `0xEA` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X1_Y1_TL = 208`
- `SYM_XHAIR_PHASE_X1_Y1_TC = 209`
- `SYM_XHAIR_PHASE_X1_Y1_TR = 210`
- `SYM_XHAIR_PHASE_X1_Y1_BL = 232`
- `SYM_XHAIR_PHASE_X1_Y1_BC = 233`
- `SYM_XHAIR_PHASE_X1_Y1_BR = 234`

### Phase `PHASE_X0_Y2` (left-bottom)

Offsets:

- `phaseX = 0`
- `phaseY = 2`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 211 | `0xD3` |
| TC | 212 | `0xD4` |
| TR | 213 | `0xD5` |
| BL | 235 | `0xEB` |
| BC | 236 | `0xEC` |
| BR | 237 | `0xED` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X0_Y2_TL = 211`
- `SYM_XHAIR_PHASE_X0_Y2_TC = 212`
- `SYM_XHAIR_PHASE_X0_Y2_TR = 213`
- `SYM_XHAIR_PHASE_X0_Y2_BL = 235`
- `SYM_XHAIR_PHASE_X0_Y2_BC = 236`
- `SYM_XHAIR_PHASE_X0_Y2_BR = 237`

### Phase `PHASE_X1_Y2` (right-bottom)

Offsets:

- `phaseX = 1`
- `phaseY = 2`

Glyphs:

| Tile | Decimal | Hex |
|---|---:|---:|
| TL | 214 | `0xD6` |
| TC | 215 | `0xD7` |
| TR | 216 | `0xD8` |
| BL | 238 | `0xEE` |
| BC | 239 | `0xEF` |
| BR | 240 | `0xF0` |

Suggested symbol names:

- `SYM_XHAIR_PHASE_X1_Y2_TL = 214`
- `SYM_XHAIR_PHASE_X1_Y2_TC = 215`
- `SYM_XHAIR_PHASE_X1_Y2_TR = 216`
- `SYM_XHAIR_PHASE_X1_Y2_BL = 238`
- `SYM_XHAIR_PHASE_X1_Y2_BC = 239`
- `SYM_XHAIR_PHASE_X1_Y2_BR = 240`

## Draw Order In Betaflight

For any selected phase, the OSD code should draw the tiles in this order:

1. top-left at `(baseX + 0, baseY + 0)`
2. top-center at `(baseX + 1, baseY + 0)`
3. top-right at `(baseX + 2, baseY + 0)`
4. bottom-left at `(baseX + 0, baseY + 1)`
5. bottom-center at `(baseX + 1, baseY + 1)`
6. bottom-right at `(baseX + 2, baseY + 1)`

So every phase is always drawn as a `3 x 2` block.

## Phase Selection Rule

Recommended runtime selection:

- `phaseX = vx % 2`
- `phaseY = vy % 3`
- `baseX = vx / 2`
- `baseY = vy / 3`

Then choose the glyph group matching `(phaseX, phaseY)` and draw it at `(baseX, baseY)`.

## Effective Meaning

This gives:

- horizontal fake resolution: `x2`
- vertical fake resolution: `x3`

For the analog OSD glyph size `12 x 18 px`, that means:

- horizontal sub-step: `6 px`
- vertical sub-step: `6 px`

So this mapping is the bridge between:

- the generated BMP font sheet
- and the future Betaflight crosshair rendering code
