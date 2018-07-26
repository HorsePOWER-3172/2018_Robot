import time

import wpilib
import wpilib.drive
from wpilib.drive import DifferentialDrive

# Gamepad axes
# TODO: dict
LEFT_WHEELS_AXIS = 1
RIGHT_WHEELS_AXIS = 5
LIFT_AXIS1 = 2  # Should be triggers
LIFT_AXIS2 = 3  # Should be triggers
POV = 11  # D-PAD port

# If difference of wheel values is <=: Average two values
WHEEL_LOCK = .1

# Initiate Deadzone values
DEFAULT_DEADZONE = .2
DEADZONES = {LEFT_WHEELS_AXIS: DEFAULT_DEADZONE,
             RIGHT_WHEELS_AXIS: DEFAULT_DEADZONE, LIFT_AXIS1: .1}

# Initiate reverse values
DEFAULT_REVERSE = True
REVERSES = {LEFT_WHEELS_AXIS: DEFAULT_REVERSE,
            RIGHT_WHEELS_AXIS: DEFAULT_REVERSE, LIFT_AXIS1: True}

# Initiate Speed caps
DEFAULT_SPEED_CAP = [.8, .8]
SPEED_CAPS = {LEFT_WHEELS_AXIS: DEFAULT_SPEED_CAP,
              RIGHT_WHEELS_AXIS: DEFAULT_SPEED_CAP, LIFT_AXIS1: [.8, 0]}

SPEED_ADJUST = .1  # Amount to inc/dec max speed by with each press
CAP_HOLD_TIME = 500  # Amount of time to hold bumpers for speed reset

# PWM ports
# TODO: Dict
LEFT_PORT = 0
LIFT_PORT_2 = 3
RIGHT_PORT = 1
LIFT_PORT = 5

GAMEPAD_NUM = 0  # This should prolly always be zero

BUTTON_PORTS = {
    "REVERSE": 1,  # Reverse wheel dir
    "LOCK": 3,  # Matches wheel speed when close
    "STOP": 2,  # Stops wheels from moving
    "SPEED": 4,  # Ignores all speed caps
    "ARM IN": 5,  # Move arm in
    "ARM OUT": 6,  # Move arm out
    "INC SPEED": [11, 90],
    "DEC SPEED": [11, 270],
    "RES SPEED": [11, 180],
    "GRAB": [11, 0]
}

ARM_SPEED_IN = 1
ARM_SPEED_OUT = 1
# Gamepad rumble length in milliseconds
RUMBLE_LENGTH = 100

AUTO_SPEED = .8
AUTO_ACCEL = .5
AUTO_DUR = 4000
ACCEL = 4

ANGLE_MARGIN = .5
ANGLE_CHANGE = .1

GYRO_ENABLE = True


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
        self.robot_lift_2 = wpilib.PWMTalonSRX(LIFT_PORT_2)

        # Initiate button stuff
        self.buttons = set()
        self.button_toggles = None
        self.pressed_buttons = {}
        self.rumble_time = None
        self.rumble_length = RUMBLE_LENGTH
        self.reset_buttons()

        # Initiate gamepad
        self.game_pad = wpilib.Joystick(GAMEPAD_NUM)

        self.max_enhancer = 0

        self.smart_dashboard = wpilib.SmartDashboard
        # auto_chooser = wpilib.SendableChooser()
        # auto_chooser.addDefault("Enable Auto", "Enable Auto")
        # auto_chooser.addObject("Disable Auto", "Disable Auto")
        self.smart_dashboard.putBoolean("Enable Auto", True)
        self.auto_start_time = None

        self.current_speed = [0, 0]
        self.last_tank = 0

        self.gyro = wpilib.ADXRS450_Gyro(0)  # TODO: Figure out channel
        self.tank_dir = None

    def reset_buttons(self):
        """Resets the values of the button toggles to default likewise
        defined here
        """
        self.button_toggles = {"REVERSE": False, "LOCK": True, "STOP": False,
                               "SPEED": False, "GRAB": False}
        self.pressed_buttons = {}
        self.rumble_time = None

    def stop(self):
        self.reset_buttons()
        self.last_tank = get_millis()
        self.drive.stopMotor()
        self.robot_lift.stopMotor()
        self.robot_lift_2.stopMotor()
        self.current_speed = [0, 0]

    # Events
    def robotInit(self):
        """Runs at the same time as __init__. Appears to be redundant in this
        case
        """
        self.stop()

    def robotPeriodic(self):
        pass

    def disabledInit(self):
        self.stop()

    def disabledPeriodic(self):
        self.stop()

    def autonomousInit(self):
        self.stop()
        self.auto_start_time = None
        self.gyro.calibrate()  # Takes five seconds
        self.stop()  # Have to reset tank time to correct acceleration

    def tank(self, left, right, accel=None):
        if left == right:
            if isinstance(self.tank_dir, type(None)):
                self.tank_dir = self.gyro.getAngle()
        else:
            self.tank_dir = None

        turn = None
        current_angle = None
        if not isinstance(self.tank_dir, type(None)):
            current_angle = self.gyro.getAngle()
            if current_angle < self.tank_dir - ANGLE_MARGIN:
                turn = "right"
            elif current_angle > self.tank_dir + ANGLE_MARGIN:
                turn = "left"

        if isinstance(accel, type(None)):
            accel = ACCEL
        speed1 = self.current_speed[0]
        speed2 = self.current_speed[1]
        if not isinstance(turn, type(None)) and GYRO_ENABLE:
            if turn == "left":
                left -= ANGLE_CHANGE
                right += ANGLE_CHANGE
            elif turn == "right":
                right -= ANGLE_CHANGE
                left += ANGLE_CHANGE

        print((round(self.tank_dir, 3) if not isinstance(self.tank_dir, type(None)) else None), (round(current_angle, 3) if not isinstance(current_angle, type(None)) else None), round(left, 3), round(right, 3), turn)

        if .4 > speed1 > 0:
            if left <= 0:
                speed1 = 0
            else:
                speed1 = .4
        if -.4 < speed1 < 0:
            if left >= 0:
                speed1 = 0
            else:
                speed1 = -.4
        if .4 > speed2 > 0:
            if right <= 0:
                speed2 = 0
            else:
                speed2 = .4
        if -.4 < speed2 < 0:
            if right >= 0:
                speed2 = 0
            else:
                speed2 = -.4

        if isinstance(accel, type(None)):
            self.drive.tankDrive(left, right)
            return None
        vel_change = ((get_millis() - self.last_tank) / 1000) * accel
        if abs(speed1 - left) < WHEEL_LOCK:
            speed1 = left
        else:
            if speed1 < left:
                speed1 += vel_change
            elif speed1 > left:
                speed1 -= vel_change
        if abs(speed2 - right) < WHEEL_LOCK:
            speed2 = right
        else:
            if speed2 < right:
                speed2 += vel_change
            elif speed2 > right:
                speed2 -= vel_change
        self.current_speed = [speed1, speed2]
        # print([round(left, 2), round(right, 2)], [round(speed1, 2), round(speed2, 2)], round(vel_change, 4))
        self.drive.tankDrive(*self.current_speed)
        # print([round(x, 2) for x in self.current_speed], round(vel_change, 2), accel)
        self.last_tank = get_millis()

    def autonomousPeriodic(self):
        if self.smart_dashboard.getBoolean("Auto Enabled", True):
            if not isinstance(self.auto_start_time, type(None)):
                # Auto enabled and running
                # percent = (get_millis() - self.auto_start_time) / AUTO_DUR
                # if percent < .5:
                #     speed = (percent/.5) * AUTO_SPEED
                # elif percent < 1:
                #     speed = ((1 - percent)/.5) * AUTO_SPEED
                # else:
                #     speed = 0
                if get_millis() >= self.auto_start_time + AUTO_DUR / 2:
                    self.tank(0, 0, AUTO_ACCEL)
                else:
                    self.tank(AUTO_SPEED, AUTO_SPEED, AUTO_ACCEL)
            else:
                # Auto enabled and not started
                self.auto_start_time = get_millis()

    def teleopInit(self):
        self.stop()
        self.drive.setSafetyEnabled(True)

    def set_rumble(self, left=True, value=1, length=RUMBLE_LENGTH):
        """Sets given rumble to given value"""
        if not left:
            self.game_pad.setRumble(wpilib.Joystick.RumbleType.kLeftRumble,
                                    value)
        else:
            self.game_pad.setRumble(wpilib.Joystick.RumbleType.kRightRumble,
                                    value)
        self.rumble_time = get_millis()
        self.rumble_length = length

    def check_rumble(self):
        """Checks if rumbling has surpassed RUMBLE_LENGTH"""
        if not isinstance(self.rumble_time, type(
                None)) and get_millis() > self.rumble_time + self.rumble_length:
            # This rumbling has gone on long enough!
            self.set_rumble(True, 0)
            self.set_rumble(False, 0)
            self.rumble_time = None

    def execute_buttons(self):
        """Check whether each button was just pressed and updates relevant
        toggle
        """
        arms = 0
        button_states = self.get_buttons()
        for button_name in BUTTON_PORTS:
            port = BUTTON_PORTS[button_name]
            angle = -1
            if isinstance(port, list):
                angle = port[1]
                port = port[0]
            if port in button_states and button_states[port] is True:
                # button was just pressed
                if button_name in self.button_toggles:
                    # needs toggled
                    if button_name == "GRAB":
                        pass
                        # print(button_name, port == POV,
                        #       self.get_raw_buttons()[POV], angle)
                    if not (port == POV and not self.get_raw_buttons()[
                                                    POV] == angle):
                        new_state = not self.button_toggles[button_name]
                        self.button_toggles[button_name] = new_state
                        self.set_rumble(new_state)
                        # print(button_name, new_state)
                elif self.get_raw_buttons()[POV] == angle:
                    # Button angle correct, check button name
                    if button_name == "INC SPEED":
                        self.max_enhancer += SPEED_ADJUST
                        self.set_rumble(True, 1)
                    elif button_name == "DEC SPEED":
                        self.max_enhancer -= SPEED_ADJUST
                        self.set_rumble(False, 1)
                    elif button_name == "RES SPEED":
                        self.max_enhancer = 0
                        self.set_rumble(True, length=200)
            elif port in button_states:
                # BUTTON BEING PRESSED
                if button_name == "ARM IN":
                    arms = max(-1, min(-(ARM_SPEED_IN), 1))
                    self.button_toggles["GRAB"] = False
                elif button_name == "ARM OUT":
                    arms = max(-1, min(ARM_SPEED_OUT, 1))
                    self.button_toggles["GRAB"] = False
        # if arms == 0 and self.button_toggles["GRAB"]:
        #     self.robot_lift_2.set(-ARM_SPEED_IN)
        # else:
        #     self.robot_lift_2.set(arms)

        self.check_rumble()

    def execute_axes(self):
        """Checks each axis and updates attached motor"""
        axes = self.get_axes()
        if not self.button_toggles["STOP"]:
            self.tank(axes[LEFT_WHEELS_AXIS], axes[RIGHT_WHEELS_AXIS])
        # self.left_motor.set(axes[LEFT_WHEELS_AXIS])
        # self.right_motor.set(axes[RIGHT_WHEELS_AXIS])

        left_trigger = axes[LIFT_AXIS1]
        right_trigger = axes[LIFT_AXIS2]
        if left_trigger > 0:
            if right_trigger <= 0:
                self.robot_lift.set(-left_trigger)
                self.robot_lift_2.set(-left_trigger)
        elif right_trigger > 0:
            self.robot_lift.set(right_trigger)
            # print(right_trigger)
        else:
            self.robot_lift.set(0)
            self.robot_lift_2.set(0)

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
                if i == RIGHT_WHEELS_AXIS:
                    if value < 0:
                        value /= .9  # Right stick does not go full forward
                        value = min(value, 1)
                if _round:
                    axes[i] = round(value, 2)
                else:
                    axes[i] = value
            except IndexError:
                break
            i += 1
        return axes

    def get_axes(self, _round=False):
        """Returns the current axes and values with deadzones and scaled"""
        axes = self.get_raw_axes(_round)
        # Correct deadzones and scale
        for axis in axes:
            if axis != LIFT_AXIS1 and axis != LIFT_AXIS2:
                if axis in DEADZONES:
                    value = axes[axis]
                    neg = -1 if value < 0 else 1
                    deadzone = DEADZONES[axis]
                    if abs(value) > deadzone:
                        value -= deadzone * neg
                        value /= 1 - deadzone
                        if not self.button_toggles[
                            "SPEED"]:  # Do not impose caps
                            value *= max(-1, min(
                                SPEED_CAPS[axis][
                                    value >= 0] + self.max_enhancer, 1))
                    else:
                        value = 0
                    if REVERSES[axis]:
                        value *= -1
                    axes[axis] = value
            else:  # TODO: Clean up
                # TRIGGERS FOR LIFT
                value = axes[axis]
                deadzone = DEADZONES[LIFT_AXIS1]
                if value > deadzone:
                    value -= deadzone
                    value /= 1 - deadzone
                    if not self.button_toggles[
                        "SPEED"]:  # Do not impose caps
                        value *= max(-1, min(
                            SPEED_CAPS[LIFT_AXIS1][
                                axis == LIFT_AXIS1] + self.max_enhancer, 1))

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
                avg = smaller + dif / 2
                axes[LEFT_WHEELS_AXIS] = avg
                axes[RIGHT_WHEELS_AXIS] = avg
                if lneg:
                    axes[LEFT_WHEELS_AXIS] *= -1
                if rneg:
                    axes[RIGHT_WHEELS_AXIS] *= -1

            # print(self.button_toggles["LOCK"], left, right,
            #       axes[LEFT_WHEELS_AXIS], axes[RIGHT_WHEELS_AXIS], dif, avg)
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
        buttons[i] = self.game_pad.getPOV(0)
        return buttons

    def get_buttons(self):
        """Returns a dictionary of bools representing whether each button was
        just pressed
        """
        raw_buttons = self.get_raw_buttons()
        for button in raw_buttons:
            being_pressed = (not raw_buttons[button] is False) and raw_buttons[button] != -1
            if button not in self.pressed_buttons and being_pressed:
                # If button not already accounted for and being pressed
                self.pressed_buttons[button] = True
            elif button in self.pressed_buttons:
                if being_pressed:
                    # Being pressed, already used
                    self.pressed_buttons[button] = False
                else:
                    # Was pressed, no longer being pressed
                    del self.pressed_buttons[button]

        return self.pressed_buttons


if __name__ == '__main__':
    wpilib.run(Robot)


# TODO: Fix smart dashboard
# TODO: Fix POV
# TODO: Test Gyro