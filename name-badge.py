#!/usr/bin/env python3

import argparse

from font_hanken_grotesk import HankenGroteskBold, HankenGroteskMedium
from font_intuitive import Intuitive
from PIL import Image, ImageDraw, ImageFont

from epd_JD79667 import Inky as InkyJD79667


print("""Inky pHAT/wHAT: Hello... my name is:

Use Inky pHAT/wHAT as a personalised name badge!

""")

def getsize(font, text):
    _, _, right, bottom = font.getbbox(text)
    return (right, bottom)

inky_display = InkyJD79667()

parser = argparse.ArgumentParser()
parser.add_argument("--name", "-n", type=str, required=True, help="Your name")
args, _ = parser.parse_known_args()

# inky_display.set_rotation(180)
# set the border color
inky_display.set_border(inky_display.epd_red)

# Figure out scaling for display size

scale_size = 1.0
padding = 0

if inky_display.resolution == (400, 300):
    scale_size = 2.20
    padding = 15

if inky_display.resolution == (600, 448):
    scale_size = 2.20
    padding = 30

if inky_display.resolution == (250, 122):
    scale_size = 1.30
    padding = -5

if inky_display.resolution == (360, 184):
    scale_size = 1.80
    padding = -5


# Create a new canvas to draw on

img = Image.new("P", inky_display.resolution)
draw = ImageDraw.Draw(img)

# Load the fonts

intuitive_font = ImageFont.truetype(Intuitive, int(22 * scale_size))
hanken_bold_font = ImageFont.truetype(HankenGroteskBold, int(35 * scale_size))
hanken_medium_font = ImageFont.truetype(HankenGroteskMedium, int(16 * scale_size))

# Grab the name to be displayed

name = args.name

# Top and bottom y-coordinates for the white strip

y_top = int(inky_display.height * (5.0 / 10.0))
y_bottom = y_top + int(inky_display.height * (4.0 / 10.0))

# Draw the red, white, and red strips

for y in range(0, y_top):
    for x in range(0, inky_display.width):
        img.putpixel((x, y), inky_display.epd_black if inky_display.colour == "black" else inky_display.epd_red)

for y in range(y_top, y_bottom):
    for x in range(0, inky_display.width):
        img.putpixel((x, y), inky_display.epd_white)

for y in range(y_bottom, inky_display.height):
    for x in range(0, inky_display.width):
        img.putpixel((x, y), inky_display.epd_black if inky_display.colour == "black" else inky_display.epd_red)

# Calculate the positioning and draw the "Hello" text
hello_text = "Hello"
hello_w, hello_h = getsize(hanken_bold_font, hello_text)
hello_x = int((inky_display.width - hello_w) / 2)
hello_y = 0 + padding
draw.text((hello_x, hello_y), hello_text, inky_display.epd_white, font=hanken_bold_font)

# Calculate the positioning and draw the "my name is" text
mynameis_text = "my name is"
mynameis_w, mynameis_h = getsize(hanken_medium_font, mynameis_text)
mynameis_x = int((inky_display.width - mynameis_w) / 2)
mynameis_y = hello_h + padding
draw.text((mynameis_x, mynameis_y), mynameis_text, inky_display.epd_white, font=hanken_medium_font)

# Calculate the positioning and draw the name text

name_w, name_h = getsize(intuitive_font, name)
name_x = int((inky_display.width - name_w) / 2)
name_y = int(y_top + ((y_bottom - y_top - name_h) / 2))
draw.text((name_x, name_y), name, inky_display.epd_black, font=intuitive_font)

# Display the completed name badge

inky_display.set_image(img)
inky_display.show()
