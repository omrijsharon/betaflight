from pathlib import Path

from PIL import Image


GLYPH_WIDTH = 12
GLYPH_HEIGHT = 18

SHEET_GLYPH_COLS = 24
SHEET_GLYPH_ROWS = 4
SHEET_WIDTH = SHEET_GLYPH_COLS * GLYPH_WIDTH
SHEET_HEIGHT = SHEET_GLYPH_ROWS * GLYPH_HEIGHT

CUSTOM_GLYPH_COLS = 9
CUSTOM_GLYPH_ROWS = 4

SPRITE_TILE_COLS = 3
SPRITE_TILE_ROWS = 2
SPRITE_WIDTH = SPRITE_TILE_COLS * GLYPH_WIDTH
SPRITE_HEIGHT = SPRITE_TILE_ROWS * GLYPH_HEIGHT

ICON_SIZE = 24
CENTER_HOLE_SIZE = 8
HOLE_START = (ICON_SIZE - CENTER_HOLE_SIZE) // 2
HOLE_END = HOLE_START + CENTER_HOLE_SIZE - 1

PHASE_X_OFFSETS = (3, 9)
PHASE_Y_OFFSETS = (0, 6, 12)

GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

VALID_LOGO_COLORS = {GREEN, BLACK, WHITE}
LOGO_WIDTH = SHEET_WIDTH - (CUSTOM_GLYPH_COLS * GLYPH_WIDTH)
LOGO_HEIGHT = SHEET_HEIGHT


def set_pixel(image: Image.Image, x: int, y: int, color: tuple[int, int, int]) -> None:
    if 0 <= x < image.width and 0 <= y < image.height:
        image.putpixel((x, y), color)


def draw_crosshair_icon(canvas: Image.Image, offset_x: int, offset_y: int) -> None:
    arm_start = 0
    arm_end_before_hole = HOLE_START - 1
    arm_start_after_hole = HOLE_END + 1
    arm_end = ICON_SIZE - 1

    vertical_white_x = offset_x + (ICON_SIZE // 2) - 1
    vertical_black_x = vertical_white_x + 1
    horizontal_white_y = offset_y + (ICON_SIZE // 2) - 1
    horizontal_black_y = horizontal_white_y + 1

    # Vertical arms: white on the left, black on the right.
    for y in range(arm_start, arm_end_before_hole + 1):
        set_pixel(canvas, vertical_white_x, offset_y + y, WHITE)
        set_pixel(canvas, vertical_black_x, offset_y + y, BLACK)

    for y in range(arm_start_after_hole, arm_end + 1):
        set_pixel(canvas, vertical_white_x, offset_y + y, WHITE)
        set_pixel(canvas, vertical_black_x, offset_y + y, BLACK)

    # Horizontal arms: white on the top, black on the bottom.
    for x in range(arm_start, arm_end_before_hole + 1):
        set_pixel(canvas, offset_x + x, horizontal_white_y, WHITE)
        set_pixel(canvas, offset_x + x, horizontal_black_y, BLACK)

    for x in range(arm_start_after_hole, arm_end + 1):
        set_pixel(canvas, offset_x + x, horizontal_white_y, WHITE)
        set_pixel(canvas, offset_x + x, horizontal_black_y, BLACK)


def build_phase_sprite(phase_x: int, phase_y: int) -> Image.Image:
    sprite = Image.new("RGB", (SPRITE_WIDTH, SPRITE_HEIGHT), GREEN)
    draw_crosshair_icon(sprite, PHASE_X_OFFSETS[phase_x], PHASE_Y_OFFSETS[phase_y])
    return sprite


def phase_block_origin(phase_x: int, phase_y: int) -> tuple[int, int]:
    phase_index = phase_y * len(PHASE_X_OFFSETS) + phase_x
    block_col = phase_index % 3
    block_row = phase_index // 3
    return block_col, block_row


def paste_phase_into_sheet(sheet: Image.Image, sprite: Image.Image, phase_x: int, phase_y: int) -> None:
    block_col, block_row = phase_block_origin(phase_x, phase_y)

    glyph_col_base = block_col * SPRITE_TILE_COLS
    glyph_row_base = block_row * SPRITE_TILE_ROWS

    for tile_row in range(SPRITE_TILE_ROWS):
        for tile_col in range(SPRITE_TILE_COLS):
            left = tile_col * GLYPH_WIDTH
            top = tile_row * GLYPH_HEIGHT
            tile = sprite.crop((left, top, left + GLYPH_WIDTH, top + GLYPH_HEIGHT))

            dst_col = glyph_col_base + tile_col
            dst_row = glyph_row_base + tile_row

            dst_x = dst_col * GLYPH_WIDTH
            dst_y = dst_row * GLYPH_HEIGHT
            sheet.paste(tile, (dst_x, dst_y))


def build_sheet() -> Image.Image:
    sheet = Image.new("RGB", (SHEET_WIDTH, SHEET_HEIGHT), GREEN)

    for phase_y in range(len(PHASE_Y_OFFSETS)):
        for phase_x in range(len(PHASE_X_OFFSETS)):
            sprite = build_phase_sprite(phase_x, phase_y)
            paste_phase_into_sheet(sheet, sprite, phase_x, phase_y)

    return sheet


def validate_logo(logo: Image.Image) -> tuple[bool, Image.Image]:
    annotated = logo.copy()
    is_valid = True

    if logo.size != (LOGO_WIDTH, LOGO_HEIGHT):
        is_valid = False

    for y in range(logo.height):
        for x in range(logo.width):
            if logo.getpixel((x, y)) not in VALID_LOGO_COLORS:
                annotated.putpixel((x, y), RED)
                is_valid = False

    return is_valid, annotated


def merge_logo_if_valid(sheet: Image.Image, script_dir: Path) -> None:
    logo_path = script_dir / "logo.bmp"
    if not logo_path.exists():
        return

    logo = Image.open(logo_path).convert("RGB")
    is_valid, annotated_logo = validate_logo(logo)

    if not is_valid:
        invalid_logo_path = script_dir / "logo_not_valid.bmp"
        annotated_logo.save(invalid_logo_path, format="BMP")
        print(f"WARNING: logo.bmp is not valid. See {invalid_logo_path}")
        return

    sheet.paste(logo, (CUSTOM_GLYPH_COLS * GLYPH_WIDTH, 0))


def build_glyph_grid_preview(sheet: Image.Image) -> Image.Image:
    preview = sheet.copy()

    for glyph_row in range(SHEET_GLYPH_ROWS):
        for glyph_col in range(SHEET_GLYPH_COLS):
            left = glyph_col * GLYPH_WIDTH
            top = glyph_row * GLYPH_HEIGHT
            right = left + GLYPH_WIDTH - 1
            bottom = top + GLYPH_HEIGHT - 1

            for x in range(left, right + 1):
                preview.putpixel((x, top), RED)
                preview.putpixel((x, bottom), RED)

            for y in range(top, bottom + 1):
                preview.putpixel((left, y), RED)
                preview.putpixel((right, y), RED)

    return preview


def main() -> None:
    script_dir = Path(__file__).parent
    output_path = script_dir / "highres_crosshair_font.bmp"
    preview_output_path = script_dir / "highres_crosshair_font_grid.bmp"
    sheet = build_sheet()
    merge_logo_if_valid(sheet, script_dir)
    sheet.save(output_path, format="BMP")
    build_glyph_grid_preview(sheet).save(preview_output_path, format="BMP")
    print(f"Saved {output_path}")
    print(f"Saved {preview_output_path}")


if __name__ == "__main__":
    main()
