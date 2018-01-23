import time

import wpilib

from wpilib.drive import DifferentialDrive

maxVel = .6
minVel = -.6
stick_range = 2
deadZone = .16
equalZone = .2
adjustAmount = 2


class Robot(wpilib.IterativeRobot):

    def __init__(self):
        """Initialize the Robot Class"""
        super().__init__()
        self.robot_drive = None
        self.game_pad = None
        self.lift = None
        self.winch = None
        self.lift_top = None
        self.lift_bottom = None

        self.left_motor = None
        self.right_motor = None

        self.unalign_button = False
        self.toggle_system_button = False
        self.agitate_button = False
        self.adjust_button = False
        self.reverse_button = False

        self.aligned = True
        self.adjusted = False
        self.reversed = False
        self.winch_enabled = False
        self.stopped = False

        self.reverse_time = 0

    def get_axes(self):
        """Return a dictionary of controller axes for debugging"""
        i = 0
        axes = {}
        while True:
            try:
                self.game_pad.getRawAxis(i)
                axes[i] = round(self.game_pad.getRawAxis(i), 2)
            except IndexError:
                break
            i += 1
        return axes

    def robotInit(self):
        """Initialize the Robot for competition start"""

        # Robot Drive
        self.left_motor = wpilib.VictorSP(1)
        self.right_motor = wpilib.Spark(0)
        self.robot_drive = DifferentialDrive(self.left_motor, self.right_motor)

        self.robot_drive.setSafetyEnabled(False)
        self.robot_drive.setExpiration(0.1)
        self.robot_drive.setMaxOutput(1)

        # Systems
        self.lift = wpilib.Talon(3)
        self.winch = wpilib.Talon(2)

        # Sensors
        self.lift_top = wpilib.DigitalInput(0)
        self.lift_bottom = wpilib.DigitalInput(1)

        # Gamepad

        self.game_pad = wpilib.Joystick(0)

    # Events
    def disabledInit(self):
        self.stop()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        """Executes at the start of teleop mode"""
        pass

    def teleopPeriodic(self):
        """Executes during teleop mode"""

        # Rumble gamepad if reverse pressed
        if self.reversed:
            rumble = wpilib.Joystick.RumbleType.kRightRumble
        else:
            rumble = wpilib.Joystick.RumbleType.kLeftRumble

        if get_millis() <= self.reverse_time + 150:
            self.game_pad.setRumble(rumble, 1)
        else:
            self.game_pad.setRumble(rumble, 0)

        # Get Button Presses
        # Reverse
        if self.game_pad.getRawButton(1):
            if not self.reverse_button:
                self.reverse()
                self.reverse_button = True
        else:
            self.reverse_button = False
        if self.game_pad.getRawButton(2):
            if not self.agitate_button:
                self.agitate()
                self.agitate_button = True
        else:
            self.agitate_button = False
        if self.game_pad.getRawButton(3):
            if not self.toggle_system_button:
                self.toggle_system()
                self.toggle_system_button = True
        else:
            self.toggle_system_button = False
        if self.game_pad.getRawButton(7):
            self.e_stop()
        if self.game_pad.getRawButton(8):
            self.resume()

        # Get button holds
        if self.game_pad.getRawButton(5):
            self.unalign()
            self.unalign_button = True
        elif self.unalign_button:
            self.unalign_button = False
            self.align()
        if self.game_pad.getRawButton(6):
            self.adjust()
            self.adjust_button = True
        elif self.adjust_button:
            self.adjust_button = False
            self.unadjust()

        if not self.stopped:

            y1 = self.game_pad.getRawAxis(1) * -1
            y2 = self.game_pad.getRawAxis(3) * -1

            left = scale(y1, maxVel, minVel, stick_range, deadZone,
                         self.aligned)
            right = scale(y2, maxVel, minVel, stick_range, deadZone,
                          self.aligned)

            # Align sticks
            if self.aligned:
                tLeft = left
                tRight = right
                if link(left, right, equalZone) != 0:
                    tLeft = link(left, right, equalZone)
                if link(right, left, equalZone) != 0:
                    tRight = link(right, left, equalZone)
                left = tLeft
                right = tRight

            right *= -1  # Right motor is backwards

            if self.reversed:
                left *= -1
                right *= -1

            if self.adjusted:
                left /= adjustAmount
                right /= adjustAmount

            print(self.get_axes())

            if left == 0 and self.left_motor.get() != 0:
                self.left_motor.set(0)
            else:
                self.left_motor.set(left)
            if right == 0 and self.right_motor.get() != 0:
                self.right_motor.set(0)
            else:
                self.right_motor.set(right)
        else:
            self.stop()

    # Actions
    def stop(self):
        if self.left_motor.get() != 0:
            self.left_motor.set(0)
        if self.right_motor.get() != 0:
            self.right_motor.set(0)

    # Commands
    def unalign(self):
        self.aligned = False
        print("Align", False)

    def align(self):
        self.aligned = True
        print("Align", True)

    def adjust(self):
        self.adjusted = True
        print("Adjust", True)

    def unadjust(self):
        self.adjusted = False
        print("Adjust", False)

    def reverse(self):
        self.reversed = not self.reversed
        print("Reverse", self.reversed)

    def agitate(self):
        print("Agitate")

    def toggle_system(self):
        self.winch_enabled = not self.winch_enabled
        print("Toggle System", self.winch_enabled)

    def e_stop(self):
        self.stopped = True
        self.stop()
        print("Emergency Stop")

    def resume(self):
        self.stopped = False
        print("Resume")


def scale(vel, maxVel, minVel, stick_range, deadZone, aligned):
    nVel = (vel / stick_range) * (maxVel - minVel)

    if not aligned:
        return nVel
    if nVel > deadZone or nVel < (-deadZone):
        if nVel < maxVel - deadZone:
            if nVel > (minVel + deadZone):
                return nVel
            else:
                if aligned:
                    return minVel
                return nVel
        else:
            if aligned:
                return maxVel
            else:
                return nVel
    else:
        return 0


def link(vel1, vel2, equalZone):
    if abs(abs(vel2) - abs(vel1)) <= equalZone:
        if vel1 < 0:
            return -(abs(vel2) + abs(vel1)) / 2
        return (abs(vel2) + abs(vel1)) / 2
    return 0


def toVel(x, y, alinged, maxVel, minVel, deadZone, equalZone):
    v = (100 - abs(x)) * (y / 100) + y
    w = (100 - abs(y)) * (x / 100) + x
    left = (v + w) / 2
    right = (v - w) / 2

    left = scale(left, maxVel, minVel, stick_range, deadZone)
    right = scale(right, maxVel, minVel, stick_range, deadZone)
    if alinged:
        tLeft = left
        tRight = right
        if link(left, right, equalZone) != 0:
            tLeft = link(left, right, equalZone)
        if link(right, left, equalZone) != 0:
            tRight = link(right, left, equalZone)
        return [tLeft, tRight]


def get_millis():
    """Return the current time in milliseconds"""
    return time.time() * 1000


if __name__ == '__main__':
    wpilib.run(Robot)
