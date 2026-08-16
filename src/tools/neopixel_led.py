import board
import neopixel

# --- Configuration ---
PIN = board.D4
NUM_PIXELS = 64
# Calculated to keep current draw safe (e.g. 0.3 for ~30% brightness)
BRIGHTNESS = 0.3
PIXEL_ORDER = neopixel.GRB  # Common pixel order: neopixel.GRB or neopixel.RGB

# Define common colors
WHITE = (255, 255, 255)
OFF = (0, 0, 0)

pixels = None


def init(pin=PIN, num_pixels=NUM_PIXELS, brightness=BRIGHTNESS, pixel_order=PIXEL_ORDER):
    global pixels
    pixels = neopixel.NeoPixel(
        pin,
        num_pixels,
        brightness=brightness,
        auto_write=False,
        pixel_order=pixel_order,
    )
    return pixels


def solid(r, g, b, start=0, end=None):
    """Set pixel range to color (default all pixels)."""
    global pixels
    if pixels is None:
        init()
    if end is None:
        end = len(pixels)
    for x in range(start, end):
        pixels[x] = (r, g, b)
    pixels.show()


def fill(r, g, b):
    """Fill all pixels with color."""
    global pixels
    if pixels is None:
        init()
    pixels.fill((r, g, b))
    pixels.show()


def clear():
    """Turn off all pixels."""
    fill(0, 0, 0)


def cleanup():
    global pixels
    if pixels is not None:
        clear()
        pixels.deinit()
        pixels = None


if __name__ == "__main__":
    init()
    solid(255, 255, 255)
    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()