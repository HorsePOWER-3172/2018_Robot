import time

import wpilib
import wpilib.drive

# Gamepad axes

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
            RIGHT_WHEELS_AXIS: DEFAULT_REVERSE, LIFT_AXIS: not DEFAULT_REVERSE,
            ARM_AXIS: DEFAULT_REVERSE}

# Initiate Speed caps
DEFAULT_SPEED_CAP = [.3, .3]
SPEED_CAPS = {LEFT_WHEELS_AXIS: DEFAULT_SPEED_CAP,
              RIGHT_WHEELS_AXIS: DEFAULT_SPEED_CAP, LIFT_AXIS: [.8, 0],
              ARM_AXIS: [1, 1]}

# PWM ports

LEFT_PORT = 0
RIGHT_PORT = 1
LIFT_PORT = 5
ARM_PORT = 3

GAMEPAD_NUM = 0  # This should prolly always be zero

# Button Ports
REVERSE_BUTTON = 1
LOCK_BUTTON = 3

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

        # Initiate arm and lift
        self.robot_lift = wpilib.PWMTalonSRX(LIFT_PORT)
        self.robot_arm = wpilib.PWMTalonSRX(ARM_PORT)

        # Initiate button stuff
        self.buttons = set()
        self.button_toggles = None
        self.button_states = {}
        self.rumble_time = None
        self.reset_buttons()

        # Initiate gamepad
        self.game_pad = wpilib.Joystick(GAMEPAD_NUM)

    def reset_buttons(self):
        """Resets the values of the button toggles to default likewise
        defined here
        """
        self.button_toggles = {"REVERSE": False, "LOCK": True}
        self.button_states = {}
        self.rumble_time = None

    def stop(self):
        """Stops all motors which are currently moving"""
        if self.left_motor.get() != 0:
            self.left_motor.set(0)
        if self.right_motor.get() != 0:
            self.right_motor.set(0)
        if self.robot_lift.get() != 0:
            self.robot_lift.set(0)
        if self.robot_arm.get() != 0:
            self.robot_arm.set(0)

    def disable(self):
        """Disables speed controllers"""
        # TODO: Fix talons not showing the robot is disabled with a solid orange light
        # TODO: Redo drive based on tank-drive example in robotpy examples repo
        #       Use safety and expiration and whatever else, figure them out
        pass

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

    def teleopPeriodic(self):

        buttons = self.get_buttons()

        if buttons[REVERSE_BUTTON]:  # Was just pressed
            # Toggle reverse state
            new_state = not self.button_toggles["REVERSE"]
            self.button_toggles["REVERSE"] = new_state
            self.set_rumble(new_state)
            print("REVERSE")

        if buttons[LOCK_BUTTON]:
            new_state = not self.button_toggles["LOCK"]
            self.button_toggles["LOCK"] = new_state
            self.set_rumble(new_state)
            print("LOCK")

        self.check_rumble()

        axes = self.get_axes()

        self.left_motor.set(axes[LEFT_WHEELS_AXIS])
        self.right_motor.set(axes[RIGHT_WHEELS_AXIS])

        self.robot_lift.set(axes[LIFT_AXIS])
        self.robot_arm.set(axes[ARM_AXIS])

    def get_raw_axes(self):
        """Return a dictionary of controller axes"""
        i = 0
        axes = {}
        while True:
            try:
                value = self.game_pad.getRawAxis(i)
                axes[i] = round(value, 2)
            except IndexError:
                break
            i += 1
        return axes

    def get_raw_buttons(self):
        """Return a dictionary of raw button states"""
        i = 1  # Controller buttons start at 1
        buttons = {}
        while i <= 10:
            try:
                value = self.game_pad.getRawButton(i)
                buttons[i] = round(value, 2)
            except IndexError:
                break
            i += 1
        return buttons

    def get_buttons(self):
        """Returns a dictionary of bools representing whether each button was
        just pressed
        """
        raw_buttons = self.get_raw_buttons()  # TODO: save button state to tell when button is first pressed rather than while button is pressed
        for button in raw_buttons:
            if button not in self.button_states:
                self.button_states[button] = False
        return self.button_states

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
            dif = (abs(left) + abs(right))
            if dif <= WHEEL_LOCK and left != 0 and right != 0:
                lneg = left < 0
                rneg = right < 0
                avg = dif / 2
                axes[LEFT_WHEELS_AXIS] = avg
                axes[RIGHT_WHEELS_AXIS] = avg
                if lneg:
                    axes[LEFT_WHEELS_AXIS] *= -1
                if rneg:
                    axes[RIGHT_WHEELS_AXIS] *= -1

            # TODO: Finish fixing locking
            print(self.button_toggles["LOCK"], left, right, axes[LEFT_WHEELS_AXIS], axes[RIGHT_WHEELS_AXIS], dif, avg)
        return axes


if __name__ == '__main__':
    wpilib.run(Robot)
