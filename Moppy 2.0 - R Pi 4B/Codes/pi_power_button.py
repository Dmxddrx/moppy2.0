
#!/usr/bin/python3

from gpiozero import Button
from signal import pause
import subprocess
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s"
)

# Physical pin 5 = BCM GPIO3
# Physical pin 6 = GND
button = Button(
    3,
    pull_up=True,
    bounce_time=0.15
)


def shutdown_pi():
    logging.info("Power button pressed. Shutting down safely.")

    subprocess.Popen([
        "/usr/bin/systemctl",
        "poweroff"
    ])


button.when_pressed = shutdown_pi

logging.info("GPIO3 power button is ready.")
pause()
