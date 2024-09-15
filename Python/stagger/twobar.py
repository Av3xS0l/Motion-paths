
from stagger import Bar, Anchor
from . import MotionStudy


class debug_Except(Exception):
    "Break Here"


class TwoBar(MotionStudy):
    def __init__(self, drive1, drive2, bar1, bar2):
        self.validate_physics(drive1, drive2, bar1, bar2)

        super().__init__(drive1, drive2, bar1, bar2)

    def get_members(self):
        return ['drive1', 'drive2', 'bar1', 'bar2']

    def set_value(self, member, parameter, value):
        if member == 'drive1':
            self.drive1.set_value(parameter, value)
        elif member == 'drive2':
            self.drive2.set_value(parameter, value)
        elif member == 'bar1':
            self.bar1.set_value(parameter, value)
        elif member == 'bar2':
            self.bar2.set_value(parameter, value)
        else:
            raise ValueError('Member does not exist!')

    @staticmethod
    def validate_physics(drive1: Anchor, drive2: Anchor, bar1: Bar, bar2: Bar):
        drive1Distance = drive1.distance_angle_from(drive2.x, drive2.y)[0]
        if (drive1Distance + drive1.r + drive2.r) >= (bar1.joint + bar2.length):
            raise ValueError('Bars too short!')
        if ((drive1Distance - drive1.r) + bar1.joint) < (bar2.length):
            raise ValueError('Bars too long!')

    def end_path(self, i):
        # drive 1
        angle1 = i * self.drive1.speed

        drive1X, drive1Y = self.drive1.base_point(angle1)
        #self.if_n(drive1X, "drive1X")
        #self.if_n(drive1Y, "drive1Y")

        # drive 2
        angle2 = i * self.drive2.speed
        #self.if_n(angle2, "angle2")
        #drive2X, drive2Y = self.drive2.base_point(angle2)
        driveLengthR, driveAngle = self.drive1.base_point_distance(
            angle1, angle2, self.drive2)
        #self.if_n(driveLengthR, "driveLengthR")
        #self.if_n(driveAngle, "driveAngle")
        angle = self.sides_to_angle(
            self.bar1.joint, driveLengthR, self.bar2.length)
        #self.if_n(angle, "angle")
        #angle = self.sides_to_angle(self.bar1.joint, 15, self.bar2.length)

        barEnd = self.line_end(
            drive1X, drive1Y, self.bar1.length, angle + driveAngle)
        if barEnd == (float("nan"), float("nan")):
            raise debug_Except
        return barEnd
