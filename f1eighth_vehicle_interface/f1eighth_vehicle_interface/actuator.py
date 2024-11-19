#!/usr/bin/env python3
import sys
from typing import Optional, TypeVar
from dataclasses import dataclass

from Adafruit_PCA9685 import PCA9685
from simple_pid import PID
import math

import rclpy
from rclpy.node import Node
from rclpy import Parameter
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport


T = TypeVar("T")
class MyPid:
    def __init__(self, Kp: T, Ki: T, Kd: T, period: float):
        self._kp = Kp
        self._ki = Ki
        self._kd = Kd
        self._period = period
        self.last_delta = 0
    
    def set_target(self, target: T):
        self.target = target
    
    def set_period(self, period: float):
        self._period = period
    
    def __call__(self, current: T) -> T:
        delta = self.target - current
        p = self._kp * delta
        i = self._ki * (delta * self.period)
        d = self._kd * ((delta - self.last_delta) / self.period)
        pid_value = p + i + d
        self.last_delta = delta
        return pid_value

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
        self.declare_parameter("kp_pwm", Parameter.Type.DOUBLE)
        self.declare_parameter("ki_pwm", Parameter.Type.DOUBLE)
        self.declare_parameter("kd_pwm", Parameter.Type.DOUBLE)
        self.declare_parameter("kp_ste", Parameter.Type.DOUBLE)
        self.declare_parameter("ki_ste", Parameter.Type.DOUBLE)
        self.declare_parameter("kd_ste", Parameter.Type.DOUBLE)

        publication_period = (
            1.0 / self.get_parameter("rate").get_parameter_value().double_value
        )
        
        if not isinstance(publication_period, float):
            raise ValueError(f"Invalid period {publication_period}")

        init_pwm = self.get_parameter("init_pwm").get_parameter_value().integer_value
        init_steer = self.get_parameter("init_steer").get_parameter_value().integer_value
        config = Config(
            init_pwm=init_pwm,
            min_pwm=self.get_parameter("min_pwm").get_parameter_value().integer_value,
            max_pwm=self.get_parameter("max_pwm").get_parameter_value().integer_value,
            init_steer=init_steer,
            min_steer=self.get_parameter("min_steer")
            .get_parameter_value()
            .integer_value,
            max_steer=self.get_parameter("max_steer")
            .get_parameter_value()
            .integer_value,
            tire_angle_to_steer_ratio=self.get_parameter("tire_angle_to_steer_ratio")
            .get_parameter_value()
            .double_value,
        )

        # PID controller parameters
        kp_pwm = self.get_parameter("kp_pwm").get_parameter_value().double_value
        ki_pwm = self.get_parameter("ki_pwm").get_parameter_value().double_value
        kd_pwm = self.get_parameter("kd_pwm").get_parameter_value().double_value
        kp_ste = self.get_parameter("kp_ste").get_parameter_value().double_value
        ki_ste = self.get_parameter("ki_ste").get_parameter_value().double_value
        kd_ste = self.get_parameter("kd_ste").get_parameter_value().double_value

        # Initialize the PID controller
        # min_speed_pid_output = config.min_pwm - config.init_pwm
        # max_speed_pid_output = config.max_pwm - config.init_pwm
        # self.speed_pid = PID(
        #     Kp=kp_pwm,
        #     Ki=ki_pwm,
        #     Kd=kd_pwm,
        #     output_limits=(min_speed_pid_output, max_speed_pid_output),
        #     sample_time=publication_period,
        # )
        self.speed_pid = MyPid(
            Kp=kp_pwm,
            Ki=ki_pwm,
            Kd=kd_pwm,
            period=publication_period,
        )

        # min_angle_pid_output = config.min_steer - config.init_steer
        # max_angle_pid_output = config.max_steer - config.init_steer
        # self.angle_pid = PID(
        #     Kp=kp_ste,
        #     Ki=ki_ste,
        #     Kd=kd_ste,
        #     output_limits=(min_angle_pid_output, max_angle_pid_output),
        #     sample_time=publication_period,
        # )
        self.angle_pid = MyPid(
            Kp=kp_ste,
            Ki=ki_ste,
            Kd=kd_ste,
            period=publication_period,
        )

        # Initialize the controller state
        state = State(
            target_speed=None,
            current_speed=None,
            target_tire_angle=None,
            current_tire_angle=None,
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
            # TwistWithCovarianceStamped,
            # "~/input/twist_with_covariance",
            Imu,
            "/mpu9250/imu_raw",
            self.imu_callback,
            1,
        )
        
        speed_subscription = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.velocity_callback,
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
        # speed = msg.twist.twist.linear.x
        # angular_speed = msg.twist.twist.angular.z
        angular_speed = msg.angular_velocity.z
        l = 0.325
        v = self.state.current_speed

        # TODO: check correctness of the code
        if v != 0:
            self.state.current_tire_angle = math.atan((angular_speed * l) / v) * 180 / math.pi

    def velocity_callback(self, msg):
        speed = msg.longitudinal_velocity
        self.state.current_speed = speed
        # self.get_logger().info(f"[v] Update speed {speed}")

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
        speed_pid_factor = 1
        # self.speed_pid.setpoint = self.state.target_speed
        self.speed_pid.set_target(self.state.target_speed)

        if self.state.target_speed is None or self.state.target_speed == 0 or self.state.current_speed is None:
            return self.config.init_pwm
        
        pid = int(round(self.speed_pid(self.state.current_speed) * speed_pid_factor))

        pwm_value = self.config.init_pwm + pid

        self.get_logger().info(f"pwm_value [{pwm_value}], [S]pid [{pid}], target [{self.state.target_speed}], current [{self.state.current_speed}], angular [{self.state.current_tire_angle}]")
        return max(min(pwm_value, 390), 370)

    def compute_steer_value(self) -> int:
        # TODO
        # - Use self.state.target_tire_angle and self.state.current_tire_angle to compute the error.
        # - Use self.config.tire_angle_to_steer_ratio to convert the tire angle and steer value on the servo.
        # - You are encouraged to add extra rules to improve the control.

        # TODO: Calculate the PID value
        angle_pid_factor = 1
        # steer_value = self.config.init_steer  # scenario 1, 2
        # return max(min(steer_value, 520), 480)
    
        angle_pid_factor = 1
        # self.angle_pid.setpoint = 0
        # self.angle_pid.setpoint = self.state.target_tire_angle
        self.angle_pid.set_target(self.state.target_tire_angle)

        if self.state.target_tire_angle is None or self.state.target_tire_angle == 0 or self.state.current_tire_angle is None:
            return self.config.init_steer
        
        steered_pid = int(round(self.angle_pid(self.state.current_tire_angle) * self.config.tire_angle_to_steer_ratio * angle_pid_factor))
        steer_value = self.config.init_steer - steered_pid

        return max(min(steer_value, 520), 480)


@dataclass
class State:
    target_speed: Optional[float]
    current_speed: Optional[float]

    target_tire_angle: Optional[float]
    current_tire_angle: Optional[float]

    # TODO
    # - Add additional state variables needed for your control algorithm


@dataclass
class Config:
    min_pwm: int
    init_pwm: int
    max_pwm: int

    min_steer: int
    init_steer: int
    max_steer: int

    tire_angle_to_steer_ratio: float


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
