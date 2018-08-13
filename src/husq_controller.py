#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16


class GestureController(object):

    # ROS initialization 
    def init(self):
        self.update_rate = 10   # Node frquency (Hz)

        #Last acelleration data received
        self.last_acc = [0,0,0]
        self.last_time = 0

        # Setup publishers & subscriber
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        rospy.Subscriber('/husqvarna/teleop/imu', Imu, self.callback_continuos_control)


    def callback_continuos_control(self, data):
        self.last_acc[0] = data.linear_acceleration.x
        self.last_acc[1] = data.linear_acceleration.y
        self.last_acc[2] = data.linear_acceleration.z
        self.last_time = data.header.stamp.secs

    # Controller starter
    def run(self):
        self.init()
        r = rospy.Rate(self.update_rate)
        while True:
            try:
                now = rospy.get_rostime()
                if (now.secs-self.last_time) < 1:
                    self.update()
                    r.sleep()
                else:
                    self.acceleration_reset()
                    self.update()
                    r.sleep()
            except KeyboardInterrupt:
                self.acceleration_reset()
                self.reset()
                break

    def acceleration_reset(self):
        self.last_acc = [0,0,0]

    # Shutdown handler
    def reset(self):
        print "\n"
        rospy.loginfo("RESETTING VELOCITY COMMANDS ON SHUTDOWN")
        self.update()

    # Mapping of acceleration to robot angular and linear velocities
    def update(self):
        if rospy.is_shutdown():
            return
        twist = Twist()
        twist.linear.x = self.last_acc[0] * 0.05
        twist.angular.z = self.last_acc[2] * 0.05
        self.pub_twist.publish(twist)


def main():
    rospy.init_node('husq_controller', disable_signals=True)
    controller = GestureController()
    controller.run()


if __name__ == '__main__':
    main()