# ~/ros2_ws/src/rover_control/rover_control/rover_control_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import os
import sys

class RoverControlNode(Node):

    def __init__(self):
        super().__init__('rover_control_node')
        self.subscription = self.create_subscription(
            String,
            'rover_commands',
            self.control_callback,
            10
        )
        self.setup_rover()

    def setup_rover(self):
        try:
            self.mh = Adafruit_MotorHAT()
            self.motor_id = 1
            self.motor2_id = 2
            self.servo_pin = 6

            self.motor = self.mh.getMotor(self.motor_id)
            self.motor2 = self.mh.getMotor(self.motor2_id)

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)

            self.speed = 200
            self.direction = Adafruit_MotorHAT.FORWARD
            self.pwm = GPIO.PWM(self.servo_pin, 50)
            self.pwm.start(7)
        
        except Exception as e:
            self.get_logger().error(f"Error setting up rover: {e}")

    def control_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        try:
            if command == "move_forward":
                self.move_forward()
            elif command == "turn_left":
                self.turn_left()
            elif command == "turn_right":
                self.turn_right()
            elif command == "stop":
                self.stop_rover()
        except Exception as e:
            self.get_logger().error(f"Error executing command {command}: {e}")
            self.stop_rover()

    def move_forward(self):
        self.motor.setSpeed(self.speed)
        self.motor2.setSpeed(self.speed)
        self.motor.run(self.direction)
        self.motor2.run(self.direction)

    def turn_left(self):
        self.pwm.ChangeDutyCycle(4.5)
        time.sleep(1)
        self.pwm.ChangeDutyCycle(7)

    def turn_right(self):
        self.pwm.ChangeDutyCycle(9.5)
        time.sleep(1)
        self.pwm.ChangeDutyCycle(7)

    def stop_rover(self):
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        self.motor2.run(Adafruit_MotorHAT.RELEASE)
        self.pwm.stop()
        GPIO.cleanup()

    def destroy_node(self):
        self.stop_rover()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
