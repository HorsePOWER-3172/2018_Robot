import wpilib
import wpilib.drive

# Gamepad axes

LEFT_AXIS = 5
RIGHT_AXIS = 1
LIFT_AXIS = 2  # Should be triggers
ARM_AXIS = 4

DEADZONE = .3  # Default deadzone value

DEADZONES = {LEFT_AXIS: DEADZONE, RIGHT_AXIS: DEADZONE, LIFT_AXIS: 0,
             ARM_AXIS: DEADZONE}

# Controller ports

LEFT_PORT = 0
RIGHT_PORT = 1
LIFT_PORT = 5
ARM_PORT = 3

GAMEPAD_NUM = 0  # This should prolly always be zero


class Robot(wpilib.IterativeRobot):

    def __init__(self):
        super().__init__()
        self.left_motor = wpilib.Spark(LEFT_PORT)
        self.right_motor = wpilib.Spark(RIGHT_PORT)
        self.robot_drive = wpilib.drive.DifferentialDrive(self.left_motor,
                                                          self.right_motor)
        self.robot_drive.setSafetyEnabled(False)
        self.robot_drive.setExpiration(0.1)
        self.robot_drive.setMaxOutput(1)

        self.robot_lift = wpilib.PWMTalonSRX(LIFT_PORT)
        self.robot_arm = wpilib.PWMTalonSRX(ARM_PORT)

        self.game_pad = wpilib.Joystick(GAMEPAD_NUM)

    def stop(self):
        if self.left_motor.get() != 0:
            self.left_motor.set(0)
        if self.right_motor.get() != 0:
            self.right_motor.set(0)

    # Events
    def disabledInit(self):
        self.stop()

    def disabledPeriodic(self):
        self.stop()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):

        axes = self.get_axes()

        self.left_motor.set(axes[LEFT_AXIS])
        self.right_motor.set(axes[RIGHT_AXIS])
        self.robot_lift.set(axes[LIFT_AXIS])
        self.robot_arm.set(axes[ARM_AXIS])

    def get_raw_axes(self):
        """Return a dictionary of controller axes"""
        i = 0
        axes = {}
        while True:
            try:
                self.game_pad.getRawAxis(i)
                axes[i] = self.game_pad.getRawAxis(i)
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
                deadzone = DEADZONES[axis]
                if abs(value) > deadzone:
                    neg = False
                    if value < 0:
                        neg = True
                    axes[axis] -= deadzone  # Shift back
                    axes[axis] /= 1 - deadzone  # Scale to deadzone range
                    if neg:
                        axes[axis] *= -1
                else:
                    axes[axis] = 0

        return axes


if __name__ == '__main__':
    wpilib.run(Robot)
