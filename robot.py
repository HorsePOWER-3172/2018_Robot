import time

import wpilib
import wpilib.drive
from wpilib.drive import DifferentialDrive


# TODO: Fix triggers, bind to lift

# Gamepad axes
# TODO: dict
LEFT_WHEELS_AXIS = 1
RIGHT_WHEELS_AXIS = 5
LIFT_AXIS = 0  # Should be triggers
ARM_AXIS = 4

# If difference of wheel values is <=: Average two values
WHEEL_LOCK = .1

# Initiate Deadzone values
DEFAULT_DEADZONE = .3
DEADZONES = {LEFT_WHEELS_AXIS: DEFAULT_DEADZONE,
             RIGHT_WHEELS_AXIS: DEFAULT_DEADZONE, LIFT_AXIS: .3,
             ARM_AXIS: DEFAULT_DEADZONE}

# Initiate reverse values
DEFAULT_REVERSE = False
REVERSES = {LEFT_WHEELS_AXIS: not DEFAULT_REVERSE,
            RIGHT_WHEELS_AXIS: DEFAULT_REVERSE, LIFT_AXIS: True,
            ARM_AXIS: False}

# Initiate Speed caps
DEFAULT_SPEED_CAP = [.3, .3]
SPEED_CAPS = {LEFT_WHEELS_AXIS: DEFAULT_SPEED_CAP,
              RIGHT_WHEELS_AXIS: DEFAULT_SPEED_CAP, LIFT_AXIS: [.8, 0],
              ARM_AXIS: [1, 1]}

# PWM ports
# TODO: Dict
LEFT_PORT = 0
RIGHT_PORT = 1
LIFT_PORT = 5
ARM_PORT = 3

GAMEPAD_NUM = 0  # This should prolly always be zero

BUTTON_PORTS = {
    "REVERSE": 1,
    "LOCK": 3,
    "STOP": 2,
    "CLIMB": 4,
}

# Gamepad rumble length in milliseconds
RUMBLE_LENGTH = 100


def get_millis():
    """Returns the current time in milliseconds"""
    return time.time() * 1000


class Robot(wpilib.IterativeRobot):

    def __init__(self):
        super().__init__()

        # Initiate up robot drive
        self.left_motor = wpilib.Spark(LEFT_PORT)
        self.right_motor = wpilib.Spark(RIGHT_PORT)

        self.drive = DifferentialDrive(self.left_motor, self.right_motor)
        self.drive.setExpiration(.1)

        # Initiate arm and lift
        self.robot_lift = wpilib.PWMTalonSRX(LIFT_PORT)
        self.robot_arm = wpilib.PWMTalonSRX(ARM_PORT)

        # Initiate button stuff
        self.buttons = set()
        self.button_toggles = None
        self.pressed_buttons = {}
        self.rumble_time = None
        self.reset_buttons()

        # Initiate gamepad
        self.game_pad = wpilib.Joystick(GAMEPAD_NUM)

    def reset_buttons(self):
        """Resets the values of the button toggles to default likewise
        defined here
        """
        self.button_toggles = {"REVERSE": False, "LOCK": True, "STOP": False,
                               "CLIMB": False}
        self.pressed_buttons = {}
        self.rumble_time = None

    def stop(self):
        self.drive.stopMotor()
        self.robot_lift.stopMotor()
        self.robot_arm.stopMotor()

    # Events
    def robotInit(self):
        """Runs at the same time as __init__. Appears to be redundant in this
        case
        """
        pass

    def robotPeriodic(self):
        pass

    def disabledInit(self):
        self.stop()

    def disabledPeriodic(self):
        self.stop()

    def autonomousInit(self):
        self.reset_buttons()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.reset_buttons()
        self.drive.setSafetyEnabled(True)

    def set_rumble(self, left=True, value=1):
        """Sets given rumble to given value"""
        if left:
            self.game_pad.setRumble(wpilib.Joystick.RumbleType.kLeftRumble,
                                    value)
        else:
            self.game_pad.setRumble(wpilib.Joystick.RumbleType.kRightRumble,
                                    value)
        self.rumble_time = get_millis()

    def check_rumble(self):
        """Checks if rumbling has surpassed RUMBLE_LENGTH"""
        if not isinstance(self.rumble_time, type(
                None)) and get_millis() > self.rumble_time + RUMBLE_LENGTH:
            # This rumbling has gone on long enough!
            self.set_rumble(True, 0)
            self.set_rumble(False, 0)
            self.rumble_time = None

    def execute_buttons(self):
        """Check whether each button was just pressed and updates relevant
        toggle
        """
        button_states = self.get_buttons()
        for button_name in BUTTON_PORTS:
            port = BUTTON_PORTS[button_name]
            if port in button_states and button_states[port]:
                new_state = not self.button_toggles[button_name]
                self.button_toggles[button_name] = new_state
                self.set_rumble(new_state)
                print(button_name, new_state)

        self.check_rumble()

    def execute_axes(self):
        """Checks each axis and updates attached motor"""
        axes = self.get_axes()
        if not self.button_toggles["STOP"]:
            self.drive.tankDrive(axes[LEFT_WHEELS_AXIS],
                                 axes[RIGHT_WHEELS_AXIS])
        # self.left_motor.set(axes[LEFT_WHEELS_AXIS])
        # self.right_motor.set(axes[RIGHT_WHEELS_AXIS])

        self.robot_lift.set(axes[LIFT_AXIS])
        self.robot_arm.set(axes[ARM_AXIS])

    def teleopPeriodic(self):
        """Runs repeatedly while robot is in teleop"""
        self.execute_buttons()
        self.execute_axes()

    def get_raw_axes(self, _round=False):
        """Return a dictionary of controller axes"""
        i = 0
        axes = {}
        while True:
            try:
                value = self.game_pad.getRawAxis(i)
                if _round:
                    axes[i] = round(value, 2)
                else:
                    axes[i] = value
            except IndexError:
                break
            i += 1
        return axes

    def get_axes(self):
        """Returns the current axes and values with deadzones and scaled"""
        axes = self.get_raw_axes()
        # Correct deadzones and scale
        for axis in axes:
            if axis in DEADZONES:
                value = axes[axis]
                neg = -1 if value < 0 else 1
                deadzone = DEADZONES[axis]
                if abs(value) > deadzone:
                    value -= deadzone * neg
                    value /= 1 - deadzone
                    if neg == -1:
                        value *= SPEED_CAPS[axis][0]
                    else:
                        if not (axis == LIFT_AXIS and self.button_toggles[
                            "CLIMB"]):
                            # If lift and climbing enabled, allow full speed down
                            # Down on the lift happens to use the max speed
                            value *= SPEED_CAPS[axis][1]
                else:
                    value = 0
                if REVERSES[axis]:
                    value *= -1
                axes[axis] = value

        if self.button_toggles["REVERSE"]:
            axes[LEFT_WHEELS_AXIS] *= -1
            axes[RIGHT_WHEELS_AXIS] *= -1
        dif, avg = None, None
        if self.button_toggles["LOCK"]:
            left = axes[LEFT_WHEELS_AXIS]
            right = axes[RIGHT_WHEELS_AXIS]
            dif = abs(abs(left) - abs(right))
            if dif <= WHEEL_LOCK and left != 0 and right != 0:
                lneg = left < 0
                rneg = right < 0
                smaller = min(abs(left), abs(right))
                avg = smaller + dif
                axes[LEFT_WHEELS_AXIS] = avg
                axes[RIGHT_WHEELS_AXIS] = avg
                if lneg:
                    axes[LEFT_WHEELS_AXIS] *= -1
                if rneg:
                    axes[RIGHT_WHEELS_AXIS] *= -1

            print(self.button_toggles["LOCK"], left, right,
                  axes[LEFT_WHEELS_AXIS], axes[RIGHT_WHEELS_AXIS], dif, avg)
        return axes

    def get_raw_buttons(self):
        """Return a dictionary of raw button states"""
        i = 1  # Controller buttons start at 1
        buttons = {}
        while i <= 10:
            # Gets ten buttons or stops at index error <-- shouldn't happen
            try:
                buttons[i] = self.game_pad.getRawButton(i)
            except IndexError:
                break
            i += 1
        return buttons

    def get_buttons(self):
        """Returns a dictionary of bools representing whether each button was
        just pressed
        """
        raw_buttons = self.get_raw_buttons()
        for button in raw_buttons:
            if button not in self.pressed_buttons and raw_buttons[button]:
                # If button not already accounted for and being pressed
                self.pressed_buttons[button] = True
            elif button in self.pressed_buttons:
                if raw_buttons[button]:
                    # Being pressed, already used
                    self.pressed_buttons[button] = False
                else:
                    # Was pressed, no longer being pressed
                    del self.pressed_buttons[button]

        return self.pressed_buttons


if __name__ == '__main__':
    wpilib.run(Robot)
