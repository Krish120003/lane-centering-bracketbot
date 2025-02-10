# INFO
# If you run into the problem that you cant install 'sysv-ipc' then do: sudo apt-get install python3-sysv-ipc
# The adaruit neopixel library doesnt work on the PI5, someone wrote this: https://pypi.org/project/Pi5Neo/
# use the MOSI pin (GPIO10) on PI5 for this script to work.
# NOTE: the neopixel strips are directional, they have an input direction and output direction. Check your wiring.
# NOTE: the Pi5Neo library is not working well, i had to patch it, in its source for the update_led function, theres a time.sleep(0.1), i just made that 0.01

from pi5neo import Pi5Neo
import time

def rainbow_cycle(neo, delay=0.1):
    colors = [
        (255, 0, 0),  # Red
        (255, 127, 0),  # Orange
        (255, 255, 0),  # Yellow
        (0, 255, 0),  # Green
        (0, 0, 255),  # Blue
        (75, 0, 130),  # Indigo
        (148, 0, 211)  # Violet
    ]
    for color in colors:
        neo.fill_strip(*color)
        neo.update_strip()
        time.sleep(delay)

neo = Pi5Neo('/dev/spidev0.0', 15, 800)
rainbow_cycle(neo)
