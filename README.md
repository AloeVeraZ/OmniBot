# OmniBot

Controller-driven three-wheel omni drive for Raspberry Pi. It keeps the supplied
800x480 pygame UI, BOARD-numbered GPIO wiring, generic Bluetooth controller axis
mapping, A-to-enable behavior, Y-to-disable behavior, and neutral-stick arming.

## One-command Raspberry Pi installation

Flash Raspberry Pi OS **with Desktop**, enable SSH, pair the Bluetooth controller,
then SSH into the Pi and run this as the normal user (do not put `sudo` first):

```bash
curl -fsSL https://raw.githubusercontent.com/AloeVeraZ/OmniBot/main/installer/install.sh | bash
```

The installer adds pygame, Raspberry Pi GPIO support, and Bluetooth support; clones
or updates `~/OmniBot`; validates the Python and drive math; enables desktop
auto-start; and reboots. It does not erase or replace existing Bluetooth pairings.

The controller must be connected at startup or may be connected later. Press A,
release it, and hold both sticks centered for 0.25 seconds. Press Y for an immediate
software stop. Disconnecting the controller also disables and stops the drive.

## Wiring and controls

GPIO mode is `BOARD`, so these are physical header pin numbers:

| Motor | IN1 | IN2 |
|---|---:|---:|
| Front | 40 | 38 |
| Left rear | 15 | 35 |
| Right rear | 12 | 16 |

The left stick controls forward, backward, left, and right translation. The right
stick's vertical axis controls rotation: push it up to turn left and down to turn
right. The right-rear motor direction is reversed in software because it is
mirrored relative to the other motors.

Every nonzero motor command starts immediately at 50% duty cycle, then scales from
50% to 75% as the stick moves farther. Releasing the stick stops the affected motor
immediately; it never slowly ramps down through the unusable range.

## Three-wheel behavior

With wheels tangent to the chassis and 120 degrees apart, exact forward/backward
motion intentionally commands the front wheel to zero while the two rear wheels
turn in opposite directions. A small strafe command makes the front motor turn.
That is correct three-wheel omni kinematics; the rollers allow the stopped wheel to
slide sideways. Pure rotation commands all three motors in the same direction.

If the robot still barely moves at 75% PWM while the lifted wheels spin quickly,
the limiting problem is electrical or mechanical, not kinematic software. Check
the motor supply under load, common Pi/driver ground, driver current capacity and
voltage drop, battery condition, wheel binding, robot weight, and motor gearing.
Do not power drive motors from the Raspberry Pi 5 V rail.

Runtime log:

```bash
tail -f ~/OmniBot/omnibot.log
```

Run manually:

```bash
~/OmniBot/run_omnibot.sh
```

Run the hardware-independent math tests:

```bash
cd ~/OmniBot && python3 -m unittest discover -s tests -v
```
