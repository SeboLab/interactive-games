import rospy
import time
import colorsys


class FistbumpHandler:
    def __init__(self, robot):
        self.robot = robot
        self.bumped = False
        self.counter = 0
        self.previous_accel = None

        self.rate = rospy.Rate(10)

    def prompt(self):
        time.sleep(0.2)
        self.robot.behavior.set_lift_height(0.5)

        self.detect_fistbump(8)

    def detect_fistbump(self, duration_s):
        time_elapsed = 0

        while not rospy.is_shutdown():
            proximity = self.robot.proximity.last_sensor_reading.distance.distance_mm

            if proximity <= 60:
                self.counter += 1
            else:
                self.counter = 0

            accel = self.robot.accel.x

            if (
                self.previous_accel is not None
                and not self.bumped
                and self.counter >= 1
                and abs(accel - self.previous_accel) > 100
            ):
                self.robot.anim.play_animation("anim_fistbump_success_01")
                self.bumped = True
                hue, sat, _ = colorsys.rgb_to_hsv(0, 153, 255)
                resp = self.robot.behavior.set_eye_color(hue, sat)

                print("Fistbumped!")
                break

            self.previous_accel = accel
            time_elapsed += 1

            if time_elapsed > 10 * duration_s:
                print("Fistbump not done")
                self.robot.behavior.set_lift_height(0.0)
                break

            self.rate.sleep()
