from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from board import SCL, SDA
import busio
from adafruit_motor import servo
import adafruit_pca9685
import RPi.GPIO as GPIO

Motor_A_EN = 17
Motor_A_Pin1 = 27
Motor_A_Pin2 = 18


def clip(x, lower, upper):
    return max(min(x, upper), lower)


class AdeeptControlNode(Node):
    def __init__(self):
        super().__init__("adeept_control_node")

        i2c_bus = busio.I2C(SCL, SDA)
        pca = adafruit_pca9685.PCA9685(i2c_bus)
        pca.frequency = 50

        self.steering_servo = servo.Servo(pca.channels[0])
        self.head_yaw_servo = servo.Servo(pca.channels[1])
        self.head_pitch_servo = servo.Servo(pca.channels[2])

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_A_EN, GPIO.OUT)
        GPIO.setup(Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_A_Pin2, GPIO.OUT)

        self.drive_motor = GPIO.PWM(Motor_A_EN, 1000)
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        self.drive_motor.start(100)
        self.drive_motor.ChangeDutyCycle(0)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            3,
        )

        self.cmd_head_sub = self.create_subscription(
            Twist,
            "cmd_head",
            self.cmd_head_callback,
            3,
        )

    def accelerate(self, speed: float) -> None:
        if speed > 0:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            self.drive_motor.ChangeDutyCycle(clip(int(speed * 100), 0, 100))
        else:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            self.drive_motor.ChangeDutyCycle(clip(int(-speed * 100), 0, 100))

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.accelerate(msg.linear.x * 1.5)
        self.steering_servo.angle = int(msg.angular.z * 90 + 90)

    def cmd_head_callback(self, msg: Twist) -> None:
        self.head_yaw_servo.angle = int((msg.angular.z * 90) + 90)
        self.head_pitch_servo.angle = int((msg.angular.y * 45) + 90)


def main(args=None):
    rclpy.init(args=args)

    node = AdeeptControlNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
