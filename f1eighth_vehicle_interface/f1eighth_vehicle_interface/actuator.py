#!/usr/bin/env python3
import sys
from typing import Optional
from dataclasses import dataclass

from Adafruit_PCA9685 import PCA9685
from simple_pid import PID

import rclpy
from rclpy.node import Node
from rclpy import Parameter
from geometry_msgs.msg import TwistWithCovarianceStamped
from autoware_auto_control_msgs.msg import AckermannControlCommand


class F1eighthActuator(Node):
    def __init__(self):
        super().__init__("f1eighth_actuator_node")

        # Register the publication rate parameter
        self.declare_parameter("i2c_address", Parameter.Type.INTEGER)
        self.declare_parameter("i2c_busnum", Parameter.Type.INTEGER)
        self.declare_parameter("pwm_freq", Parameter.Type.INTEGER)
        self.declare_parameter("min_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("init_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("max_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("min_steer", Parameter.Type.INTEGER)
        self.declare_parameter("init_steer", Parameter.Type.INTEGER)
        self.declare_parameter("max_steer", Parameter.Type.INTEGER)
        self.declare_parameter("tire_angle_to_steer_ratio", Parameter.Type.DOUBLE)
        self.declare_parameter("rate", Parameter.Type.DOUBLE)
        self.declare_parameter("kp", Parameter.Type.DOUBLE)
        self.declare_parameter("ki", Parameter.Type.DOUBLE)
        self.declare_parameter("kd", Parameter.Type.DOUBLE)

        publication_period = (
            1.0 / self.get_parameter("rate").get_parameter_value().double_value
        )

        config = Config(
            init_pwm=self.get_parameter("init_pwm").get_parameter_value().integer_value,
            min_pwm=self.get_parameter("min_pwm").get_parameter_value().integer_value,
            max_pwm=self.get_parameter("max_pwm").get_parameter_value().integer_value,
            init_steer=(
                self.get_parameter("init_steer").get_parameter_value().integer_value
            ),
            min_steer=self.get_parameter("min_steer")
            .get_parameter_value()
            .integer_value,
            max_steer=self.get_parameter("max_steer")
            .get_parameter_value()
            .integer_value,
            tire_angle_to_steer_ratio=self.get_parameter("tire_angle_to_steer_ratio")
            .get_parameter_value()
            .double_value,
            period=publication_period,
        )

        # PID controller parameters
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value

        # Initialize the PID controller
        min_pid_output = config.min_pwm - config.init_pwm
        max_pid_output = config.max_pwm - config.init_pwm
        self.speed_pid = PID(
            Kp=kp,
            Ki=ki,
            Kd=kd,
            output_limits=(min_pid_output, max_pid_output),
        )

        # Initialize the controller state
        state = State(
            target_speed=None,
            current_speed=None,
            target_tire_angle=None,
            current_tire_angle=None,
            last_speed_delta=None,
            last_angle_delta=None,
        )

        # Initialize the PCA9685 driver
        pwm_freq = self.get_parameter("pwm_freq").get_parameter_value().integer_value
        i2c_address = (
            self.get_parameter("i2c_address").get_parameter_value().integer_value
        )
        i2c_busnum = (
            self.get_parameter("i2c_busnum").get_parameter_value().integer_value
        )
        driver = PCA9685(address=i2c_address, busnum=i2c_busnum)
        driver.set_pwm_freq(pwm_freq)

        # Subscribe to control commands
        control_cmd_subscription = self.create_subscription(
            AckermannControlCommand,
            "~/input/control_cmd",
            self.control_callback,
            1,
        )

        # Subscribe to IMU data
        imu_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            "~/input/twist_with_covariance",
            self.imu_callback,
            1,
        )

        # Start periodic calls
        timer = self.create_timer(publication_period, self.timer_callback)

        # Save variables
        self.config = config
        self.state = state
        self.control_cmd_subscription = control_cmd_subscription
        self.imu_subscription = imu_subscription
        self.driver = driver
        self.timer = timer

    def imu_callback(self, msg):
        speed = msg.twist.twist.linear.x
        angular_speed = msg.twist.twist.angular.z

        self.state.current_speed = speed

        # TODO: add tire_angle_to_steer_ratio
        self.state.current_tire_angle = angular_speed

    def control_callback(self, msg):
        self.state.target_speed = msg.longitudinal.speed
        self.state.target_tire_angle = msg.lateral.steering_tire_angle

    def timer_callback(self):
        # Set the power of the DC motor
        pwm_value = self.compute_pwm_value()
        self.driver.set_pwm(0, 0, pwm_value)

        # Set angle of the steering servo
        steer_value = self.compute_steer_value()
        self.driver.set_pwm(1, 0, steer_value)

    def compute_pwm_value(self) -> int:
        # TODO
        # - Use self.state.target_speed and self.state.current_speed to compute the error.
        # - Compute the PID controller output that will be added to init_pwm
        # - You are encouraged to add extra rules to improve the control.

        # TODO: Calculate the PID value
        delta = self.state.target_speed - self.state.current_speed
        p = self.speed_pid.Kp * delta
        i = self.speed_pid.Ki * (delta * self.config.period)
        d = self.speed_pid.Kd * ((delta - self.state.last_speed_delta) / self.config.period)
        pid_value = p + i + d
        self.state.last_speed_delta = delta

        pwm_value = self.config.init_pwm + pid_value
        return pwm_value

    def compute_steer_value(self) -> int:
        # TODO
        # - Use self.state.target_tire_angle and self.state.current_tire_angle to compute the error.
        # - Use self.config.tire_angle_to_steer_ratio to convert the tire angle and steer value on the servo.
        # - You are encouraged to add extra rules to improve the control.

        # TODO: Calculate the PID value
        delta = self.state.target_tire_angle - self.state.current_tiretarget_tire_angle
        p = self.speed_pid.Kp * delta
        i = self.speed_pid.Ki * (delta * self.config.period)
        d = self.speed_pid.Kd * ((delta - self.state.last_angle_delta) / self.config.period)
        steer_value = (p + i + d) * self.config.tire_angle_to_steer_ratio
        self.state.last_angle_delta = delta

        return steer_value


@dataclass
class State:
    target_speed: Optional[float]
    current_speed: Optional[float]

    target_tire_angle: Optional[float]
    current_tire_angle: Optional[float]

    # TODO
    # - Add additional state variables needed for your control algorithm
    last_speed_delta: Optional[float]
    last_angle_delta: Optional[float]


@dataclass
class Config:
    min_pwm: int
    init_pwm: int
    max_pwm: int

    min_steer: int
    init_steer: int
    max_steer: int

    tire_angle_to_steer_ratio: float

    period: float


def main():
    rclpy.init(args=sys.argv)
    node = F1eighthActuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully shutdown on keyboard interrupt
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
