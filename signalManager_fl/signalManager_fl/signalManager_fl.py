# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import struct
from enum import Enum


import board
import busio
import RPi.GPIO as GPIO
import adafruit_mcp4725
import math

from std_msgs.msg import Int16
from std_msgs.msg import Byte
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import smbus
from mpu6050 import mpu6050
import time


class State(Enum):
    ERROR = 0
    INACTIVE = 1
    WAITING = 2
    NORMAL = 3

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('signalManager_fl')
        self.publisher_heartBeat = self.create_publisher(Byte, 'forklift1/heartbeat/signalManager_fl', 10)
        self.publisher_vibration = self.create_publisher(Float32, 'forklift1/arduino/vibration', 10)
        self.subscription = self.create_subscription(Bool, 'forklift1/light/front_laser', self.laser_callback, 10)
        self.subscription1 = self.create_subscription(Int16, 'forklift1/target/throttle', self.throttle_callback, 1)
        self.subscription2 = self.create_subscription(Int16, 'forklift1/target/steering', self.steering_callback, 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.safety_counter = 0
        self.max_safety_counter = 20


        self.target_throttle = 0
        self.actual_throttle = 0
        self.max_delta_throttle = 1000
        self.target_steering = 180
        self.actual_steering = 180
        self.max_delta_steering = 5
        self.relais = False
        self.relais_pin = 'SOC_GPIO54'
        GPIO.setup(self.relais_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(self.relais_pin, GPIO.HIGH)
        print("GPIOs")
        i2c = busio.I2C(board.SCL, board.SDA)
        i2c1 = busio.I2C(board.SCL_1, board.SDA_1)
        print("I2Cs")
        self.throttle_pedal = adafruit_mcp4725.MCP4725(i2c1, address=0x60)
        self.steering_sin = adafruit_mcp4725.MCP4725(i2c, address=0x60)
        self.steering_cos = adafruit_mcp4725.MCP4725(i2c, address=0x61)
        print("mcp")
        self.dac_status_throttle = True
        self.dac_status_steering = True
        self.vibration_history = []
        self.init_vibration()
        print("initVibration")
        self.get_logger().info('init completed')
        #ros2 topic pub -r 10 /forklift1/arduino/throttle std_msgs/msg/Int16 '{data: 500}'
        #ros2 topic pub -r 10 /forklift1/arduino/steering std_msgs/msg/Int16 '{data: 60}'
        #ros2 topic pub -r 10 /forklift1/light/front_laser std_msgs/msg/Bool '{data: true}'

    def init_vibration(self):
        self.imu = mpu6050(0x68, bus=1)
        self.imu.set_filter_range(0x00) # 0x06=5Hz 0x00=256Hz
        self.accelerometer_data = self.imu.get_accel_data()
        sum_x = 0
        sum_y = 0
        sum_z = 0
        n = 100
        for i in range(n): 
            accelerometer_data = self.imu.get_accel_data()
            sum_x += accelerometer_data['x']
            sum_y += accelerometer_data['y']
            sum_z += accelerometer_data['z']
            time.sleep(0.01)
        self.offset_x = sum_x/n
        self.offset_y = sum_y/n
        self.offset_z = sum_z/n
        self.offset = math.sqrt(self.offset_x**2 + self.offset_y**2 + self.offset_z**2)
        for i in range(5):
            self.vibration_history.append(self.get_vibration())
            time.sleep(0.01)
 
    def laser_callback(self, msg):
        self.get_logger().info('subscribed laser: '+ str(msg.data))
    
    def throttle_callback(self, msg):
        #self.get_logger().info('subscribed throttle: '+ str(msg.data))
        self.target_throttle = msg.data
        self.safety_counter = 0

    def steering_callback(self, msg):
        #self.get_logger().info('subscribed steering: '+ str(msg.data))
        self.target_steering = msg.data
    
    def set_target_steering(self, target_steering_):
        if target_steering_ > self.actual_steering + self.max_delta_steering:
            self.actual_steering = self.actual_steering + self.max_delta_steering
        elif target_steering_ < self.actual_steering - self.max_delta_steering:
            self.actual_steering = self.actual_steering - self.max_delta_steering
        else:
            self.actual_steering = target_steering_
        self.set_steering_wheel(self.actual_steering)
    

    def set_target_throttle(self, target_throttle_):
        if target_throttle_ > self.actual_throttle + self.max_delta_throttle:
            self.actual_throttle = self.actual_throttle + self.max_delta_throttle
        elif target_throttle_ < self.actual_throttle - self.max_delta_throttle:
            self.actual_throttle = self.actual_throttle - self.max_delta_throttle
        else:
            self.actual_throttle = target_throttle_
        self.set_throttle_pedal(self.actual_throttle)
    

    def set_steering_wheel(self, value):
        raw_value_sin = 2048 + (int) (1638 * (math.sin(math.radians(value)))) #2.5V + 2V * sin(winkel)
        raw_value_cos = 2048 + (int) (1638 * (math.cos(math.radians(value)))) #2.5V + 2V * cos(winkel)
        self.dac_status_steering = True
        try:
            self.steering_sin.raw_value = raw_value_sin
            self.steering_cos.raw_value = raw_value_cos
        except OSError:
            self.get_logger().error('i2c error steering')
            self.dac_status_steering = False
        #print('SIN: ' + str(raw_value_sin*5/4095))
        #print('COS: ' + str(raw_value_cos*5/4095))

    def set_throttle_pedal(self, value):
        raw_value = 246 + (int) (3849 * value / 1000)
        self.dac_status_throttle = True
        try:
            self.throttle_pedal.raw_value = raw_value
        except OSError:
            self.get_logger().error('i2c error throttle')
            self.dac_status = False
        if self.raw_value_to_voltage(raw_value) > 0.6:
            GPIO.output(self.relais_pin, GPIO.LOW)
            self.relais = True
        elif self.raw_value_to_voltage(raw_value) < 0.5:
            GPIO.output(self.relais_pin, GPIO.HIGH)
            self.relais = False
            

    def raw_value_to_voltage(self, raw_value):
        return raw_value/4095*5


    """ def get_target_steering(self):
        self.accelerometer_data = self.imu.get_accel_data()
        diff_x = self.accelerometer_data['x']-last_x
        diff_y = self.accelerometer_data['y']-last_y
        diff_z = self.accelerometer_data['z']-last_z
        diff = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        self.vibration = diff
        if(diff>0.1):
            print(diff)
        self.last_x = self.accelerometer_data['x']
        self.last_y = self.accelerometer_data['y']
        self.last_z = self.accelerometer_data['z'] """

    def get_vibration(self):
        self.accelerometer_data = self.imu.get_accel_data()
        x = self.accelerometer_data['x'] - self.offset_x
        y = self.accelerometer_data['y'] - self.offset_y
        z = self.accelerometer_data['z'] - self.offset_z
        self.vibration = math.sqrt(x**2 + y**2 + z**2)
        return self.vibration

    def timer_callback(self):
        try:
            self.safety_counter += 1
            if self.safety_counter > self.max_safety_counter:
                self.target_throttle = 0
            self.set_target_throttle(self.target_throttle)
            self.set_target_steering(self.target_steering)
            #self.get_target_steering()
            self.get_vibration()
            self.vibration_history.pop()
            self.vibration_history.append(self.vibration)

            msg = Float32()
            avg = sum(self.vibration_history)/len(self.vibration_history)
            if avg > 0.15:
                print(str(avg))
            msg.data = avg
            self.publisher_vibration.publish(msg)
            
            msg = Byte()
            if self.dac_status_steering and self.dac_status_throttle:
                msg.data = b'3' #State.NORMAL
            else:
                msg.data = b'0'
            self.publisher_heartBeat.publish(msg)
        except:            
            GPIO.output(self.relais_pin, GPIO.HIGH)
            self.relais = False





def main(args=None):

    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
