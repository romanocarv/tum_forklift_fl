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
import serial
import struct
from enum import Enum

from std_msgs.msg import Int16
from std_msgs.msg import Byte
from std_msgs.msg import Bool

laser_topic = 1
sensor_topic = 2
throttle_topic = 3


class State(Enum):
    ERROR = 0
    INACTIVE = 1
    WAITING = 2
    NORMAL = 3

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('arduinoManager_fl')
        self.publisher_frontDistance = self.create_publisher(Int16, 'forklift1/sensor/front_distance', 10)
        self.publisher_heartBeat = self.create_publisher(Byte, 'forklift1/heartbeat/arduinoManager_fl', 10)
        self.subscription = self.create_subscription(Bool, 'forklift1/light/front_laser', self.laser_callback, 10)
        self.subscription = self.create_subscription(Int16, 'forklift1/arduino/throttle', self.throttle_callback, 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
        self.get_logger().info('init completed')
    
    def laser_callback(self, msg):
        self.write_topic(laser_topic, int(msg.data))
    
    def throttle_callback(self, msg):
        self.get_logger().info('subscribed: '+ str(msg.data))
        self.write_topic(throttle_topic, int(msg.data))
    
    def read(self):
        id_int = None
        data_int = None
        while self.arduino.in_waiting > 0 and self.arduino.readable:
            b = self.arduino.read(1)
            if b == b'>':
                #print('end message')
                break
            if b != b'<':
                continue
            id = self.arduino.read(2)
            id_int = int.from_bytes(id, "little")
            data = self.arduino.read(2)
            data_int = int.from_bytes(data, "little")
            endByte = self.arduino.read(1)
            if endByte != b'>':
                return None, None
            #print('topic id: ' + str(id_int) + ', data: ' + str(data_int))
        return id_int, data_int
    
    def write_topic(self, topic, data):
            b = self.arduino.write(b'[')
            self.arduino.write(topic.to_bytes(2, 'little'))
            self.arduino.write(data.to_bytes(2, 'little'))
            b = self.arduino.write(b']')

    def timer_callback(self):
        topic, data = self.read()

        if topic is not None:
            if topic == sensor_topic:
                print('data' + str(data))
                msg = Int16()
                msg.data = data
                self.publisher_frontDistance.publish(msg)
        #self.write_topic(123, 531)
        msg = Byte()
        msg.data = b'3' #State.NORMAL
        self.publisher_heartBeat.publish(msg)


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
