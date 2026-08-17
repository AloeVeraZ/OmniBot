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

## Positional servo on HAT channel 0

The supported HAT is the 16-channel Waveshare-style PCA9685 I2C Servo Driver HAT
at its default `0x40` address. Connect a 360-degree **positional** servo to channel
0 with the ground, 5V, and signal wires in the board's indicated orientation.

- Hold the left trigger to move at maximum servo speed toward `-150 degrees`.
- Hold the right trigger to move at maximum servo speed toward `+150 degrees`.
- Release both triggers to hold the current position.
- Press X to return to `0 degrees`.

The goBILDA 25-2 Torque servo is a 300-degree positional servo, so the software
never commands beyond `-150..+150 degrees`. Its documented full PWM span is
500-2500 microseconds. Before attaching the mechanism, test the servo unloaded and
calibrate `SERVO_MIN_PULSE_US`,
`SERVO_CENTER_PULSE_US`, and `SERVO_MAX_PULSE_US` in `servo_hat.py`. Immediately
reduce the pulse range if the servo buzzes, heats, or pushes against a physical
stop. The installer enables I2C and installs the SMBus driver automatically.

## Wiring and controls

GPIO mode is `BOARD`, so these are physical header pin numbers:

| Motor | IN1 | IN2 |
|---|---:|---:|
| Front | 40 | 38 |
| Left rear | 15 | 35 |
| Right rear | 12 | 16 |

The left stick controls forward, backward, left, and right translation. Its
horizontal input is inverted to match this controller. The right stick's horizontal
axis controls rotation and is also inverted so right means right. Rotation is
accepted only while the right stick remains inside a narrow vertical center band;
up, down, and diagonal input do nothing. Both rear motor directions are reversed in
software relative to the front motor to match the physical 120-degree chassis.

Every nonzero motor command applies a 0.20-second 100% breakaway pulse to overcome
static friction, then runs between 75% and 100% as the stick moves farther. Full
stick stays at a true 100% duty cycle. Releasing the stick stops the affected motor
immediately; it never slowly ramps down through the unusable range.

## Three-wheel behavior

With wheels tangent to the chassis and 120 degrees apart, exact forward/backward
motion intentionally commands the front wheel to zero while the two rear wheels
turn in opposite directions. A small strafe command makes the front motor turn.
That is correct three-wheel omni kinematics; the rollers allow the stopped wheel to
slide sideways. Pure rotation commands all three motors in the same direction.

If the robot still barely moves at 100% PWM while the lifted wheels spin quickly,
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
