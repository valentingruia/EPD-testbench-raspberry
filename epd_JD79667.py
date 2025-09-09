"""Inky e-Ink Display Driver."""
""" Teste with: """
""" - GDEY0266F52H produced by Dalian Good Display Co., Ltd."""
"""                res 184(H)x360V                          """
"""                colors: black, white, red, yellow        """
import time
import warnings

import numpy
import gpiod
from gpiod.line import Bias, Direction, Value
from PIL import Image


RESET_PIN = 27  # GPIO27 - PIN13
BUSY_PIN = 13   # GPIO13 - PIN11
DC_PIN = 22     # GPIO22 - PIN15

MOSI_PIN = 10
SCLK_PIN = 11
CS_PIN = 8      # GPIO08 - PIN14

JD79668_PSR = 0x00
JD79668_PWR = 0x01
JD79668_POF = 0x02
JD79668_POFS = 0x03
JD79668_PON = 0x04
JD79668_BTST_P = 0x06
JD79668_DSLP = 0x07
JD79668_DTM = 0x10
JD79668_DRF = 0x12
JD79668_CDI = 0x50
JD79668_TCON = 0x60
JD79668_TRES = 0x61
JD79668_PWS = 0xE3

_SPI_CHUNK_SIZE = 4096

#                       (width, height)
_RESOLUTION_2_66_INCH = (360, 184)

_RESOLUTION = {
    _RESOLUTION_2_66_INCH: (_RESOLUTION_2_66_INCH[0], _RESOLUTION_2_66_INCH[1], 0, 0, 0, 0b01),
}


class Inky:
    """Inky e-Ink Display Driver."""

    # 2bit
    epd_black  = 0x00
    epd_white  = 0x01
    epd_yellow = 0x02
    epd_red    = 0x03

    DESATURATED_PALETTE = [
        [0, 0, 0],
        [255, 255, 255],
        [255, 255, 0],
        [255, 0, 0]]

    SATURATED_PALETTE = [
        [13, 13, 13],
        [71, 71, 71],
        [81, 62, 10],
        [42, 13, 10]]

    def __init__(self, resolution=None, colour="red/yellow", cs_pin=CS_PIN, dc_pin=DC_PIN, reset_pin=RESET_PIN, busy_pin=BUSY_PIN, h_flip=False, v_flip=False, spi_bus=None, gpio=None):  # noqa: E501
        """Initialise an Inky Display.

        :param resolution: (width, height) in pixels, default: (800, 480)
        :param colour: one of red, black or yellow, default: black
        :param cs_pin: chip-select pin for SPI communication
        :param dc_pin: data/command pin for SPI communication
        :param reset_pin: device reset pin
        :param busy_pin: device busy/wait pin
        :param h_flip: enable horizontal display flip, default: False
        :param v_flip: enable vertical display flip, default: False

        """

        self._spi_bus = spi_bus


        # Check for supported display variant and select the correct resolution
        if resolution is None:
            resolution = _RESOLUTION_2_66_INCH

        if resolution not in _RESOLUTION.keys():
            raise ValueError(f"Resolution {resolution[0]}x{resolution[1]} not supported!")

        self.resolution = resolution
        self.width, self.height = resolution
        self.cols, self.rows, self.rotation, self.offset_x, self.offset_y, self.resolution_setting = _RESOLUTION[resolution]

        self.border_colour = self.epd_white
        self._epd_colors = [self.epd_white, self.epd_yellow, self.epd_red, self.epd_black]

        if colour not in ("red/yellow"):
            raise ValueError(f"Colour {colour} is not supported!")

        self.colour = colour
        self.lut = colour

        self.buf = numpy.zeros((self.rows, self.cols), dtype=numpy.uint8)

        self.dc_pin = dc_pin
        self.reset_pin = reset_pin
        self.busy_pin = busy_pin
        self.cs_pin = cs_pin
        try:
            self.cs_channel = [8, 7].index(cs_pin)
        except ValueError:
            self.cs_channel = 0
        self.h_flip = h_flip
        self.v_flip = v_flip

        self._gpio = gpio
        self._gpio_setup = False

        self._luts = None

    def _palette_blend(self, saturation, dtype="uint8"):
        saturation = float(saturation)
        palette = []
        for i in range(4):
            rs, gs, bs = [c * saturation for c in self.SATURATED_PALETTE[i]]
            rd, gd, bd = [c * (1.0 - saturation) for c in self.DESATURATED_PALETTE[i]]
            if dtype == "uint8":
                palette += [int(rs + rd), int(gs + gd), int(bs + bd)]
            if dtype == "uint24":
                palette += [(int(rs + rd) << 16) | (int(gs + gd) << 8) | int(bs + bd)]
        return palette

    def setup(self):
        """Set up Inky GPIO and reset display."""
        if not self._gpio_setup:
            if self._gpio is None:
                with gpiod.Chip('/dev/gpiochip0') as chip:
                    self._gpio = chip.request_lines(consumer="inky", config={
                        self.cs_pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE, bias=Bias.DISABLED),
                        self.dc_pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE, bias=Bias.DISABLED),
                        self.reset_pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE, bias=Bias.DISABLED),
                        self.busy_pin: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
                    })

            if self._spi_bus is None:
                import spidev
                self._spi_bus = spidev.SpiDev()

            self._spi_bus.open(0, self.cs_channel)
            self._spi_bus.max_speed_hz = 1000000

            self._gpio_setup = True

        # reset active while low-INACTIVE, high-ACTIVE
        self._gpio.set_value(self.reset_pin, Value.INACTIVE)
        time.sleep(0.04)
        self._gpio.set_value(self.reset_pin, Value.ACTIVE)
        time.sleep(0.05)

        self._busy_wait()
        self._send_command(0x4D, [0x78])
        self._send_command(JD79668_PSR, [0x0F, 0x29])
        self._send_command(JD79668_PWR, [0x07, 0x00])
        self._send_command(JD79668_POFS, [0x10, 0x54, 0X44])
        self._send_command(JD79668_BTST_P, [0x05, 0x00, 0x3F, 0x0A, 0x25, 0x12, 0x1A])
        self._send_command(JD79668_CDI, [0x37])
        self._send_command(JD79668_TCON, [0x02, 0x02])
        high_heigh = (self.height >> 8) & 0xFF
        low_heigh  = self.height & 0xFF
        high_width = (self.width >> 8) & 0xFF
        low_width  = self.width & 0xFF
        self._send_command(JD79668_TRES, [high_heigh, low_heigh, high_width, low_width])
        self._send_command(0xE7, [0x1C])
        self._send_command(0xE3, [0x22])
        self._send_command(0xB4, [0xD0, 0xB5, 0x03])
        self._send_command(0xE9, [0x01])
        self._send_command(0x30, [0x08])
        self._send_command(0x04)
        self._busy_wait()

    def _busy_wait(self, timeout=0.5):
        """Wait for busy/wait pin."""
        # If the busy_pin is *high* (pulled up by host)
        # then assume we're not getting a signal from inky
        # and wait the timeout period to be safe.
        # if self._gpio.get_value(self.busy_pin) == Value.ACTIVE:
        #    time.sleep(timeout)
        #    return

        print("Busy pin", self._gpio.get_value(self.busy_pin))

        t_start = time.time()
        while self._gpio.get_value(self.busy_pin) == Value.ACTIVE:
            time.sleep(0.05)
            if time.time() - t_start > timeout:
                warnings.warn(f"Busy Wait: Timed out after {timeout:0.2f}s")
                return

    def _update(self, buf):
        """Update display.

        Dispatches display update to correct driver.

        """
        self.setup()

        self._send_command(JD79668_DTM, buf)
        # power ON
        #  self._send_command(JD79668_PON)
        self._busy_wait()
        self._send_command(JD79668_DRF, [0x00])
        self._busy_wait()
        # power off
        self._send_command(JD79668_POF, [0x00])
        self._busy_wait()
        # deep sleep
        self._send_command(JD79668_DSLP, [0xA5])
        self._busy_wait()

    def _color_test_vertical(self):
        # Vertical, Horizontal
        pix_resolution = self.resolution[0]*self.resolution[1]
        buf = bytearray(pix_resolution//4)

        pix_resolution //= 16
        for section in range(0, 4):
            epd_color = self._epd_colors[section]
            color = (epd_color<<6) | (epd_color<<4) | (epd_color<<2) | epd_color
            for idx in range(section*pix_resolution, (section+1)*pix_resolution):
                buf[idx] = color
        # display the image
        self._update(buf)

    def _color_test_horizontal(self):
        # Vertical, Horizontal
        pix_resolution = self.resolution[0]*self.resolution[1]
        buf = bytearray(pix_resolution//4)
        # 4 bytes per byte & display is divided in 4 horizontal segments
        bytes_horiz= self.resolution[0]
        bytes_vert = self.resolution[1] // 4
        count = 10
        for section in range(0, 4):
            epd_color = self._epd_colors[section]
            color = (epd_color<<6) | (epd_color<<4) | (epd_color<<2) | epd_color
            for idx_v in range((section*bytes_vert)//4, ((section+1)*bytes_vert)//4):
                for idx_h in range(idx_v, idx_v+bytes_vert*bytes_horiz, bytes_vert):
                    buf[idx_h] = color

        # signal pixel 0
        for i in range(0,5):
            for j in range(0,2):
                buf[i*bytes_vert+j] = (self.epd_black<<6) | (self.epd_black<<4) | (self.epd_black<<2) | self.epd_black

        # display the image
        self._update(buf)


    def set_pixel(self, x, y, v):
        """Set a single pixel.

        :param x: x position on display
        :param y: y position on display
        :param v: colour to set

        """

        if v in (self.epd_white, self.epd_yellow, self.epd_red, self.epd_black):
            self.buf[y][x] = v

    def show(self, busy_wait=True):
        """Show buffer on display.

        :param busy_wait: If True, wait for display update to finish before returning.

        """
        region = self.buf

        if self.v_flip:
            region = numpy.fliplr(region)

        if self.h_flip:
            region = numpy.flipud(region)

        if self.rotation:
            region = numpy.rot90(region, self.rotation // 90)

        buf = region.flatten()

        print(f'buf len {len(buf)}')

#        self._color_test_horizontal()

        bin_array = bytearray(len(buf))
        for idx in range(0, len(buf)):
            widths, reminder = divmod(idx, self.width)
            # ok bin_array[widths + reminder*self.height] = buf[idx]
            bin_array[widths + (self.width-reminder-1)*self.height] = buf[idx]

        buf = [ ((a & 0x03) << 6) |
                ((b & 0x03) << 4) |
                ((c & 0x03) << 2) |
                (d & 0x03)
                for a, b, c, d in zip(bin_array[::4], bin_array[1::4], bin_array[2::4], bin_array[3::4])]
        print(f'buf len sec {len(buf)}')

        self._update(buf)


        # data_list = bytearray(len(buf))
        # for temp in buf:
        #     data_H1 = self._epd_colors[(temp >> 6) & 0x03] << 6
        #     data_H2 = self._epd_colors[(temp >> 4) & 0x03] << 4
        #     data_L1 = self._epd_colors[(temp >> 2) & 0x03] << 2
        #     data_L2 = self._epd_colors[temp & 0x03]
        #     data_list.append(data_H1 | data_H2 | data_L1 | data_L2)
        # self._update(data_list)


    def set_border(self, colour):
        """Set the border colour."""
        if colour in (self.epd_white, self.epd_yellow, self.epd_red, self.epd_black):
            self.border_colour = colour

    def set_image(self, image, saturation=0.5):
        """Copy an image to the display.

        :param image: PIL image to copy, must be 400x300
        :param saturation: Saturation for quantization palette - higher value results in a more saturated image

        """
        if not image.size == (self.width, self.height):
            raise ValueError(f"Image must be ({self.width}x{self.height}) pixels!")

        dither = Image.Dither.FLOYDSTEINBERG

        # Image size doesn't matter since it's just the palette we're using
        palette_image = Image.new("P", (1, 1))

        if image.mode == "P":
            # Create a pure colour palette from DESATURATED_PALETTE
            palette = numpy.array(self.DESATURATED_PALETTE, dtype=numpy.uint8).flatten().tobytes()
            palette_image.putpalette(palette)

            # Assume that palette mode images with an unset palette use the
            # default colour order and "DESATURATED_PALETTE" pure colours
            if not image.palette.colors:
                image.putpalette(palette)

            # Assume that palette mode images with exactly four colours use
            # all the correct colours, but not exactly in the right order.
            if len(image.palette.colors) == 4:
                dither = Image.Dither.NONE
        else:
            # All other image should be quantized and dithered
            palette = self._palette_blend(saturation)
            palette_image.putpalette(palette)

        image = image.convert("RGB").quantize(4, palette=palette_image, dither=dither)
        # image.save("img1.png","PNG")

        self.buf = numpy.array(image, dtype=numpy.uint8).reshape((self.rows, self.cols))

    def _send_command(self, command, data=None):
        """Send command over SPI.
        :param command: command byte
        :param data: optional list of values
        """

        self._gpio.set_value(self.cs_pin, Value.INACTIVE)
        self._gpio.set_value(self.dc_pin, Value.INACTIVE)
        time.sleep(0.3)
        self._spi_bus.xfer3([command])

        if data is not None:
            self._gpio.set_value(self.dc_pin, Value.ACTIVE)
            self._spi_bus.xfer3(data)

        self._gpio.set_value(self.cs_pin, Value.ACTIVE)
        self._gpio.set_value(self.dc_pin, Value.INACTIVE)
